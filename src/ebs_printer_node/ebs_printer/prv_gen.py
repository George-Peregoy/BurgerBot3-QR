#!/usr/bin/env python

"""
Renders a .prv preview thumbnail for a .prj's barcode Text, using the same
qrcode-library approach as qrcode_gen.py (repo root) -- a QRCode with
error_correction=ERROR_CORRECT_L, matching that script's qr_test().

Format confirmed against a genuine printer-generated .prv
(Printer/EBS-exportFile-test-qr/Projects/test_qr.prv): it's a 1:1 pixel
rendering of the whole project canvas (ProjectSettings w/h), an indexed
(2-color) PNG, with the QR code occupying exactly the barcode Object's
x/y/w/h box. For that project (Object w=h=21), that box is a version-1
QR -- version 1 is always 21x21 modules -- rendered at 1px/module with
no border, which is what this reproduces. Unlike qrcode_gen.py's
qr_test() (version=1, make(fit=False), raises DataOverflowError if the
text doesn't fit), this uses make(fit=True) and scales the result to
the Object's box, so longer Text values degrade gracefully instead of
hard-failing.
"""

import argparse
import xml.etree.ElementTree as et

import qrcode
from PIL import Image


def _barcode_object_geometry(prj_path):
    """(text, canvas_w, canvas_h, x, y, w, h) from prj_path's first BarcodeObject."""
    root = et.parse(prj_path).getroot()
    settings = root.find('ProjectSettings')
    canvas_w, canvas_h = int(settings.get('w')), int(settings.get('h'))
    obj = root.find(".//Object[@ObjectType='BarcodeObject']")
    if obj is None:
        raise ValueError('%r has no BarcodeObject to preview' % prj_path)
    return (
        obj.get('Text'),
        canvas_w, canvas_h,
        int(obj.get('x')), int(obj.get('y')), int(obj.get('w')), int(obj.get('h')),
    )


def generate_prv(prj_path, output_path=None):
    """Build a .prv preview PNG for prj_path, matching the real printer
    format: an indexed-color PNG sized to the project canvas, with a QR
    code rendering of the barcode Object's Text positioned at its x/y/w/h.

    Returns the path written to.
    """
    text, canvas_w, canvas_h, x, y, w, h = _barcode_object_geometry(prj_path)

    qr = qrcode.QRCode(error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=1, border=0)
    qr.add_data(text)
    qr.make(fit=True)
    qr_img = qr.make_image(fill_color='black', back_color='white').convert('L')
    qr_img = qr_img.resize((w, h), Image.NEAREST)

    canvas = Image.new('L', (canvas_w, canvas_h), color=255)
    canvas.paste(qr_img, (x, y))
    canvas = canvas.convert('P', palette=Image.ADAPTIVE, colors=2)

    if output_path is None:
        output_path = prj_path.rsplit('.', 1)[0] + '.prv'
    canvas.save(output_path, format='PNG')
    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prj', help='path to the .prj file to build a preview for')
    parser.add_argument('-o', '--output', help='output .prv path (default: <prj>.prv)')
    args = parser.parse_args()

    path = generate_prv(args.prj, args.output)
    print('Wrote %s' % path)


if __name__ == '__main__':
    main()
