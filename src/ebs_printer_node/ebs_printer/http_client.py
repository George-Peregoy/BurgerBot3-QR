#!/usr/bin/env python

"""
HTTP client for the EBS printer's web control interface.

Talks directly to the same endpoints the browser UI calls
(cgi-bin/ebs-www-manager), instead of driving the UI through a browser
like EBS_Bot in ebs_print_final.py does. The CMD codes and parameter
names below were read out of the shipped UI's own JS -- see==
"""

import os
import socket
import time
# from abc import ABC, abstractmethod

import requests

# CMD codes, taken verbatim from the shipped UI's JS.
CMD_LOGIN = 20450          # users.js CMD_LOGIN
CMD_OPEN_PROJECT = 11010   # project.js P_OPEN_EXISTING_PROJECT (OpenProjectForPrinter)
CMD_START_PRINT = 10000    # printerCmd.js PRINTER_START_PRINT
CMD_STOP_PRINT = 10002     # printerCmd.js PRINTER_ISTOP_PRINT -- what the UI's Stop
                            # button actually sends, despite PRINTER_STOP_PRINT (10001)
                            # also existing as a separate code
CMD_CHECK_STATUS = 10010   # printerCmd.js PRINTER_CHECK_STATUS


class EBSError(Exception):
    """Base class for all errors raised by this module.

    Lets callers catch "anything printer-related" with a single
    `except EBSError:` while still allowing precise handling of a
    specific failure mode via the subclasses below.
    """


class EBSAuthError(EBSError):
    """Login failed, or a session could not be (re-)established."""


class EBSCommandError(EBSError):
    """The printer answered with a non-OK Status for a command."""


class EBSProtocolError(EBSError):
    """Response wasn't parseable JSON -- most likely a session drop that
    got us redirected to an HTML login page instead of the CGI JSON reply."""


def _local_ip_for(remote_ip):
    """Best-effort local interface IP used to reach `remote_ip`.

    Mirrors the browser-supplied `RemoteAddr` login param: the UI embeds
    the viewer's own IP server-side at page load time, e.g.
    `top.RemoteAddr = "192.168.10.196"` in the saved index page. We have
    no server to do that for us, so we ask the OS which local address
    would be used to route to the printer.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((remote_ip, 80))
        return sock.getsockname()[0]
    finally:
        sock.close()


# class EBSPrinterClient(ABC):
#     """Common interface for talking to the EBS printer.

#     EBSHttpClient below is the primary implementation. If the CGI protocol
#     turns out to be incomplete for print control (plan step 3), a
#     Selenium-backed implementation (see EBS_Bot in ebs_print_final.py) can
#     be adapted to this same interface and swapped in without touching
#     callers.
#     """

#     @abstractmethod
#     def login(self, force=False):
#         pass

#     @abstractmethod
#     def upload(self, prj_path, remote_dir="/Projects"):
#         pass

#     @abstractmethod
#     def select(self, project_path):
#         pass

#     @abstractmethod
#     def start(self, force_print=0):
#         pass

#     @abstractmethod
#     def stop(self):
#         pass

#     @abstractmethod
#     def status(self):
#         pass


class EBSHttpClient():
    """requests-based client for the EBS printer's cgi-bin/ebs-www-manager API."""

    def __init__(self, ip, user_id="1", password="1", timeout=10):
        self.ip = ip
        self.user_id = user_id
        self.password = password
        self.timeout = timeout
        self.base_url = "http://%s" % ip
        self.session = requests.Session()
        # project_import.php (and presumably other .php pages) render every
        # UI string as empty -- not an error page, just blank labels -- when
        # there's no Accept-Language header to pick a language table from.
        # python-requests sends none by default; confirmed via a byte-diff
        # against a working Chrome capture, whose only header difference on
        # an otherwise-identical request was Accept-Language.
        self.session.headers.update({"Accept-Language": "en-US,en;q=0.9"})
        self.user_level = None
        self.logged_in = False

    # -- low-level ---------------------------------------------------

    def _cmd(self, cmd, **params):
        """POST a CMD to cgi-bin/ebs-www-manager and return the parsed JSON.

        The endpoint expects space-separated `key=value` pairs in the
        body, not `&` -- confirmed in project.js, e.g.
        `data: "CMD="+CMD+" printerIP="+printerIP`. requests won't build
        that encoding for us, so the body is assembled by hand.
        """
        parts = ["CMD=%s" % cmd]
        for key, value in params.items():
            parts.append("%s=%s" % (key, requests.utils.quote(str(value), safe="")))
        body = " ".join(parts)

        resp = self.session.post(
            "%s/cgi-bin/ebs-www-manager" % self.base_url,
            data=body,
            headers={"Content-Type": "application/x-www-form-urlencoded; charset=utf-8"},
            timeout=self.timeout,
        )
        resp.raise_for_status()
        try:
            return resp.json()
        except ValueError:
            raise EBSProtocolError(
                "Non-JSON response for CMD=%s -- likely redirected to a login "
                "page (session expired?)" % cmd
            )

    @staticmethod
    def _is_auth_failure(answer):
        status = answer.get("Status", "")
        return status.startswith("Error:Login") or status in (
            "Error:NotLoggedIn",
            "Error:SessionExpired",
            "Logout",
        )

    def _cmd_authed(self, cmd, **params):
        """Like _cmd, but re-logs in and retries once on an auth failure."""
        try:
            answer = self._cmd(cmd, **params)
            if self._is_auth_failure(answer):
                raise EBSProtocolError("Status=%s" % answer.get("Status"))
            return answer
        except EBSProtocolError:
            self.login(force=True)
            answer = self._cmd(cmd, **params)
            if self._is_auth_failure(answer):
                raise EBSAuthError(
                    "Re-login did not restore the session (Status=%s)" % answer.get("Status")
                )
            return answer

    # -- session -------------------------------------------------------

    def login(self, force=False):
        """Authenticate against the printer, mirroring users.js LoginUser().

        Raises EBSAuthError on failure. On success, stashes the same
        LoggedUser*/UserLevel values the browser UI would store as
        cookies via `$.cookie(...)` -- the CGI response itself doesn't
        set them via Set-Cookie, the client is expected to.
        """
        remote_addr = _local_ip_for(self.ip)
        answer = self._cmd(
            CMD_LOGIN,
            id=self.user_id,
            password=self.password,
            forceLogin=1 if force else 0,
            RemoteAddr=remote_addr,
        )

        status = answer.get("Status")
        if status != "OK":
            if status == "Error:SomeoneLoggedIn" and not force:
                return self.login(force=True)
            raise EBSAuthError("Login failed: %s" % status)

        self.user_level = answer.get("UserLevel")
        self.session.cookies.set("LoggedUserName", str(answer.get("LoggedUserName", "")))
        self.session.cookies.set("LoggedUserIP", str(answer.get("LoggedUserIP", "")))
        self.session.cookies.set("LoggedUserID", str(answer.get("LoggedUserID", "")))
        self.session.cookies.set("UserLevel", str(self.user_level))
        self.logged_in = True
        return answer

    # -- project lifecycle ----------------------------------------------

    def upload(self, exp_path):
        """Upload a project export (.exp) bundle via project_import.php.

        Confirmed via live traffic (wireshark_sessions/manual_import_test.pcapng):
        this is a plain multipart POST -- not a CGI CMD -- with exactly two
        fields, `install=1` (hidden) and `importFile=<file>`. There is no
        `Directory` field. The response is the import tab's HTML re-rendered
        (not JSON); success shows up as the literal text
        "Project <filename> imported" in that HTML.

        `exp_path` must point to a .exp file, which is a ZIP archive --
        not a raw .prj (see exp_build.py's build_exp()).

        project_import.php is a plain PHP page, unlike the CGI binary
        login() talks to -- it depends on browser-session state login()
        never establishes (it never GETs any .php page). Confirmed via
        four live captures (script_failed_upload_test/test2/test3/
        GET_test.pcapng), ruling out causes one at a time:
        - Not the .exp's zip structure -- uploading a byte-identical
          copy of a known-good .exp still failed the same way.
        - Not a missing Accept-Language header -- adding it changed
          nothing.
        - Not a missing PHPSESSID cookie -- GETting project_import.php
          before POSTing to it (added below) correctly picks one up via
          Set-Cookie and carries it into the POST, but the response was
          still every localized UI string blank (including the
          "imported" success text this method checks for) -- and that
          same blank-string response even came back on the GET itself,
          before any POST.
        - Current suspect: `EBS-2500_Editor_Version`, a cookie the
          working browser capture has that we never picked up, only
          ever `Set-Cookie`'d on a logged-in `GET /index.php` (confirmed
          in manual_test.pcapng) -- a page this client has never once
          visited. The page's own markup branches on editor/product
          version (there's a whole separate "EBS 250/230" import form
          alongside the one we use), so a missing version cookie
          plausibly breaks its ability to pick a text bundle. Untested
          against the real printer yet.
        """
        self.session.get("%s/index.php" % self.base_url, timeout=self.timeout)
        self.session.get("%s/include/project_import.php" % self.base_url, timeout=self.timeout)

        filename = os.path.basename(exp_path)
        with open(exp_path, "rb") as f:
            files = {"importFile": (filename, f, "application/octet-stream")}
            resp = self.session.post(
                "%s/include/project_import.php" % self.base_url,
                files=files,
                data={"install": "1"},
                timeout=self.timeout,
            )
        resp.raise_for_status()

        if "Project %s imported" % filename not in resp.text:
            looks_like_login_page = "login" in resp.url.lower() or "password" in resp.text.lower()
            raise EBSCommandError(
                "Upload failed or unconfirmed for %r (final URL: %s, likely a "
                "login redirect: %s, status %s, response starts: %r)"
                % (filename, resp.url, looks_like_login_page, resp.status_code, resp.text[:300])
            )
        return resp.text

    def select(self, project_path, different_print_head_open=0):
        """Open `project_path` for printing. Mirrors project.js
        OpenProjectForPrinter().

        Confirmed via live traffic (manual_test.pcapng): ProjectPath is
        root-relative within the printer's own project store, e.g.
        '/bot_test.prj' -- there is no 'Projects' directory prefix on
        the wire, even though the status endpoint's CurrentProject field
        echoes it back as 'projects/bot_test.prj'.
        """
        answer = self._cmd_authed(
            CMD_OPEN_PROJECT,
            DifferentPrintHeadOpen=different_print_head_open,
            ProjectPath=project_path,
        )
        if answer.get("Status") != "OK":
            raise EBSCommandError(
                "Could not open project %r: %s" % (project_path, answer.get("Status"))
            )
        return answer

    def start(self, force_print=0):
        """Start printing the currently-selected project."""
        answer = self._cmd_authed(CMD_START_PRINT, forcePrint=force_print)
        if answer.get("Status") not in ("OK", "MustEdit"):
            raise EBSCommandError("Start print failed: %s" % answer.get("Status"))
        return answer

    def stop(self):
        """Stop printing."""
        return self._cmd_authed(CMD_STOP_PRINT)

    def status(self):
        """Return the printer's current status JSON (CurrentProject, IsPrinting, ...)."""
        return self._cmd_authed(CMD_CHECK_STATUS)

    def print_project(self, project_path, duration=None, force_print=0, different_print_head_open=0):
        """select() -> start() -> (wait `duration` seconds, if given) -> stop().

        Prototype built from the select/start/stop sequence captured in
        wireshark_sessions/manual_test.pcapng -- CMD codes and params
        confirmed there, but this specific combined method is untested
        against real hardware (no printer available while writing it).

        `project_path` is the same wire-format string select() takes,
        e.g. '/TEST4.prj' -- root-relative, no 'Projects/' prefix, no
        relation to any local .prj/.exp file path. For a project built
        via exp_build.py, that's '/' + the .prj's filename.

        `duration` is required to decide when to stop, not optional
        busywork: this is a triggered/continuous marking printer, not a
        one-shot job with a natural end -- the project's own
        PrintingParams (PhotocellSource/TriggerType/ImpulseGeneratorSource)
        describe a physical trigger, and in manual_test.pcapng,
        status()'s IsPrinting/StatusCode fields (48/D0100) never changed
        once across an entire start-to-stop cycle, nor across a second
        one right after -- there is no known signal in status() to
        auto-detect completion. So start() most likely just arms the
        printer to fire on trigger; it stays armed until stopped,
        however many trigger events fire in between. If `duration` is
        None, this selects+starts and returns immediately, leaving the
        printer armed -- call stop() yourself when ready.
        """
        self.select(project_path, different_print_head_open=different_print_head_open)
        answer = self.start(force_print=force_print)
        if duration is not None:
            time.sleep(duration)
            self.stop()
        return answer
