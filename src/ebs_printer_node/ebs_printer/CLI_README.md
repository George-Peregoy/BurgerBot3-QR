# ebs_printer

A standalone, ROS-independent Python client for the EBS industrial
printer's web control interface (specifically EBS-260). Generates a barcode/QR project, packages
it, uploads it, and can select/start/stop a print — all over plain HTTP.

> For the debugging history behind these design decisions (why things are
> shaped the way they are), see [`../SESSION_NOTES.md`](../SESSION_NOTES.md).

## Setup

```bash
cd Printer
python3 -m venv .venv && source .venv/bin/activate
pip install -e .
```

`-e` (editable) means changes to this package's source are picked up
immediately, no reinstall needed. This is also how to install
`ebs_printer` into a ROS2 workspace's Python environment (system
Python, no venv) so a node can `import ebs_printer` — see
[`pyproject.toml`](../pyproject.toml) for the declared dependencies.

## Quick start: the CLI

```bash
python -m ebs_printer.cli "some text"
```

This runs the whole pipeline end to end: generates a `.prj` encoding
`"some text"` as a QR/barcode, compresses it into a `.exp`, logs in,
uploads it, then selects it, starts printing, waits, and stops.

**This controls a physical print head.** Only run it while at the
printer and able to watch what it does.

### CLI options

| Flag | Default | What it does |
|---|---|---|
| `text` (positional) | — | Text to encode in the barcode/QR object |
| `--ip` | `192.168.10.102` | Printer's IP address |
| `--user-id` | `1` | Login user ID |
| `--password` | `1` | Login password |
| `--fonts-dir` | none | Directory to pull `fonts/User/*.ttf` files the project references from (currently always `MTCORSVA.ttf`). If omitted, that font is assumed to already be installed on the printer. |
| `--outdir` | `.` | Where to write the generated `.prj`/`.exp`/`.prv` |
| `--duration` | `2.0` | Seconds to leave the print armed before stopping. See [caveat](#the-duration-flag-is-a-guess-not-a-completion-check) below. |
| `--skip-print` | off | Generate and upload only — don't touch `select`/`start`/`stop` |

Examples:

```bash
# Just build and upload, don't print
python -m ebs_printer.cli "batch-42" --skip-print

# Leave it armed for 10 seconds before stopping
python -m ebs_printer.cli "batch-42" --duration 10

# Different printer, with the font bundled explicitly
python -m ebs_printer.cli "batch-42" --ip 192.168.1.50 --fonts-dir ./fonts
```

## What each piece does

| Module | Job |
|---|---|
| [`prj_gen.py`](prj_gen.py) | Builds a `.prj` (the printer's project XML format) whose barcode object encodes a given text string. The printer renders the actual barcode itself via `zint` at print time — this file doesn't generate any image. |
| [`prv_gen.py`](prv_gen.py) | Renders the `.prv` preview thumbnail (a small indexed PNG) for a `.prj`, using the `qrcode` library. Needed for the project to display correctly on the printer, not just be present. |
| [`exp_build.py`](exp_build.py) | Packages a `.prj` (+ auto-generated `.prv`, + any referenced fonts) into a flat `.exp` ZIP — the format `project_import.php` accepts. |
| [`http_client.py`](http_client.py) | `EBSHttpClient` — the actual HTTP client. Login, upload, select, start, stop, status, all as plain `requests` calls against the printer's CGI binary (`cgi-bin/ebs-www-manager`) and `project_import.php`. |
| [`cli.py`](cli.py) | Wires all of the above together as `python -m ebs_printer.cli`. |

Each of `prj_gen.py`, `prv_gen.py`, and `exp_build.py` can also be run on
its own, as a submodule (not as a bare script — see below):

```bash
python -m ebs_printer.prj_gen "some text" -o out.prj
python -m ebs_printer.exp_build out.prj -o out.exp --fonts-dir ./fonts
```

> **Note:** these modules use relative imports (they're part of the
> `ebs_printer` package), so you must run them as `python -m
> ebs_printer.<module>` from the `Printer/` directory — running
> `python3 ebs_printer/exp_build.py` directly will fail with an
> `ImportError`.

## Using it as a library

```python
from ebs_printer.http_client import EBSHttpClient
from ebs_printer.exp_build import build_exp
from ebs_printer.prj_gen import generate_prj

prj_path = generate_prj("some text", "some_text.prj")
exp_path = build_exp(prj_path)

client = EBSHttpClient("192.168.10.102")
client.login()
client.upload(exp_path)
client.print_project("/some_text.prj", duration=2)
```

`EBSHttpClient`'s methods:

- `login(force=False)` — authenticate; auto-retries with `force=True` if
  someone else is already logged in.
- `upload(exp_path)` — upload a `.exp` bundle.
- `select(project_path, different_print_head_open=0)` — open an
  already-uploaded project for printing. `project_path` is root-relative,
  e.g. `"/some_text.prj"` — no `Projects/` prefix, regardless of the
  `.exp`'s internal layout.
- `start(force_print=0)` / `stop()` — start/stop printing.
- `status()` — current printer status (ink, battery, current project).
- `print_project(project_path, duration=None, ...)` — `select` + `start`
  + (wait `duration`s + `stop`, if given). See caveat below.

All raise `EBSError` subclasses (`EBSAuthError`, `EBSCommandError`,
`EBSProtocolError`) on failure.

## Caveats

### The `duration` flag is a guess, not a completion check

This is a triggered/continuous marking printer (it fires on a physical
sensor per the project's `PrintingParams`), not a one-shot "print one
document and stop" printer. There is no known field in `status()` that
distinguishes "printing" from "idle" — in testing, `IsPrinting` and
`StatusCode` stayed identical across an entire start-to-stop cycle.
`start()` most likely just arms the printer; `--duration`/`duration=`
is a plain wall-clock wait before calling `stop()`, not a real
completion signal. Pass `duration=None` (or `--skip-print`, for the CLI)
if you want to leave the printer armed and call `stop()` yourself.

### Ink/consumable faults aren't currently surfaced

A `{"Status": "OK"}` response from `start()` means the command was
*accepted*, not that the printer physically printed anything — an
expired ink cartridge, for instance, won't show up as an error here.
