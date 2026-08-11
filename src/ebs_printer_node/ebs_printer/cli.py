#!/usr/bin/env python

"""
Standalone debug harness for the EBS printer pipeline: generate a
barcode/QR project, upload it, and print it -- entirely outside ROS.

    python -m ebs_printer.cli "some text"

Exists specifically to keep ebs_printer usable and testable on its own,
independent of whatever ROS 2 node eventually wraps it (see
Printer/SESSION_NOTES.md's rewrite plan) -- and as a quick way to smoke
test the printer without going through the ROS layer at all.
"""

import argparse
import os

from .exp_build import build_exp
from .http_client import EBSHttpClient
from .prj_gen import generate_prj

DEFAULT_IP = "192.168.10.102"


def run(text, ip=DEFAULT_IP, user_id="1", password="1", fonts_dir=None,
        outdir=".", duration=2.0, skip_print=False):
    """generate -> upload -> (select -> start -> wait `duration`s -> stop).

    Returns the logged-in EBSHttpClient, so callers using this as a
    library function (not just via main()) can keep issuing commands
    (status(), another print_project(), ...) afterwards.
    """
    prj_path = generate_prj(text, os.path.join(outdir, "%s.prj" % text))
    print("Generated %s" % prj_path)

    exp_path = build_exp(prj_path, fonts_dir=fonts_dir)
    print("Packaged %s" % exp_path)

    client = EBSHttpClient(ip, user_id=user_id, password=password)
    print(client.login())
    print(client.upload(exp_path))

    if not skip_print:
        project_path = "/" + os.path.basename(prj_path)
        print(client.print_project(project_path, duration=duration))

    return client


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("text", help="text to encode in the barcode/QR project")
    parser.add_argument("--ip", default=DEFAULT_IP, help="printer IP (default: %(default)s)")
    parser.add_argument("--user-id", default="1", help="login user id (default: %(default)s)")
    parser.add_argument("--password", default="1", help="login password (default: %(default)s)")
    parser.add_argument("--fonts-dir", help="directory to look up referenced fonts/User/*.ttf files in")
    parser.add_argument(
        "--outdir", default=".",
        help="directory to write the generated .prj/.exp/.prv to (default: current directory)",
    )
    parser.add_argument(
        "--duration", type=float, default=2.0,
        help="seconds to leave the print armed before stopping (default: %(default)s)",
    )
    parser.add_argument(
        "--skip-print", action="store_true",
        help="generate and upload only -- don't select/start/stop",
    )
    args = parser.parse_args()

    run(
        args.text,
        ip=args.ip,
        user_id=args.user_id,
        password=args.password,
        fonts_dir=args.fonts_dir,
        outdir=args.outdir,
        duration=args.duration,
        skip_print=args.skip_print,
    )


if __name__ == "__main__":
    main()
