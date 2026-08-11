#!/usr/bin/env python

"""
Packages a .prj into a .exp zip bundle for upload via project_import.php.

Flat layout: 'Projects/<name>.prj' and 'Fonts/<font>' at the zip root,
no wrapping top-level directory. Confirmed via a controlled live-traffic
comparison, not just code-reading:

- TEST123.exp (flat, this layout, no explicit dir entries) -- imported
  and is actually visible/selectable on the printer's project list.
- TEST2.exp (wrapped under 'TEST2_export/', with explicit dir entries --
  a prior version of this file) -- project_import.php's response said
  "Project TEST2.exp imported" (a real success response, not the
  blank-text bug), but the project never appeared on the printer.

So the wrapper directory isn't just unnecessary, it actively breaks
placement: the server accepts and confirms the upload either way, but
only the flat layout actually lands the files where the project browser
looks. (A wrapped .exp, EBS-exportTest.exp, was captured succeeding
once before this comparison existed -- but "succeeding" there only ever
meant the confirmation text, which we now know doesn't imply the file
ended up visible. It was never independently confirmed to show up on
the printer.)
"""

import argparse
import os
import xml.etree.ElementTree as et
import zipfile

from .prv_gen import generate_prv

USER_FONT_PREFIX = 'fonts/User/'


def _referenced_fonts(prj_path):
    """User-font basenames referenced by prj_path's Objects.

    Mirrors export.php's font-bundling check:
    `strpos($Object['FontName'], "fonts/User/") === 0`.
    """
    root = et.parse(prj_path).getroot()
    fonts = []
    for obj in root.iter('Object'):
        font_name = obj.get('FontName')
        if font_name and font_name.startswith(USER_FONT_PREFIX):
            basename = font_name[len(USER_FONT_PREFIX):]
            if basename not in fonts:
                fonts.append(basename)
    return fonts


def build_exp(prj_path, output_path=None, fonts_dir=None):
    """Zip prj_path (+ its .prv preview, generated via prv_gen.py if it
    doesn't already exist, + any referenced user fonts found in
    fonts_dir) into a .exp bundle ready for EBSHttpClient.upload().

    Fonts the project references but that aren't found in fonts_dir are
    skipped rather than failing the build -- the printer likely already
    has them installed.

    Only fonts are handled here, not ImageObject/VariableFManager files
    that export.php also bundles -- prj_gen.py doesn't generate those
    object types, so that path is untested and left out.
    """
    prj_path = os.path.abspath(prj_path)
    prj_name = os.path.basename(prj_path)
    if output_path is None:
        output_path = os.path.splitext(prj_path)[0] + '.exp'

    with zipfile.ZipFile(output_path, 'w', zipfile.ZIP_DEFLATED) as zf:
        zf.write(prj_path, 'Projects/' + prj_name)

        prv_path = os.path.splitext(prj_path)[0] + '.prv'
        if not os.path.exists(prv_path):
            generate_prv(prj_path, prv_path)
        zf.write(prv_path, 'Projects/' + os.path.basename(prv_path))

        for font_basename in _referenced_fonts(prj_path):
            font_path = os.path.join(fonts_dir, font_basename) if fonts_dir else None
            if font_path and os.path.exists(font_path):
                zf.write(font_path, 'Fonts/' + font_basename)
            else:
                print(
                    'Warning: font %r referenced but not found in fonts_dir -- '
                    'assuming it is already installed on the printer' % font_basename
                )

    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prj', help='path to the .prj file to package')
    parser.add_argument('-o', '--output', help='output .exp path (default: <prj>.exp)')
    parser.add_argument('--fonts-dir', help='directory to look up referenced fonts/User/*.ttf files in')
    args = parser.parse_args()

    path = build_exp(args.prj, args.output, args.fonts_dir)
    print('Wrote %s' % path)


if __name__ == '__main__':
    main()
