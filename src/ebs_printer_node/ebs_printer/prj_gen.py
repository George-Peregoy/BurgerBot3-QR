#!/usr/bin/env python

"""
Generates an EBS printer project (.prj) file whose barcode object
encodes a given text string. The printer renders the barcode image
itself via zint (see the Object 'Cmd' attribute below), so no QR
image needs to be generated on our end.

PROJECT_SETTINGS.w and BARCODE_OBJECT.AutoSize/Transparent were
originally guessed from the shipped UI's JS, not a real project. A
project built with the old values (AutoSize=0, Transparent=1, w=200)
imported onto the printer successfully but rendered blank there.
Diffing against a genuine printer-exported project
(Printer/EBS-exportFile-test-qr/Projects/test_qr.prj) found those three
values differed from a project confirmed to render correctly; adjusted
to match.
"""

import argparse
import xml.etree.ElementTree as et
from xml.dom import minidom

PROJECT_SETTINGS = {
    'DataFormat': '10',
    'PrintHeadName': 'EBS-Electromagnetic-32',
    'FriendlyName': 'Undefined',
    'w': '1000',
    'h': '32',
    'ObjectsCount': '1',
    'ManagerInfo': '1',
    'min_w': '51',
}

PRINTING_PARAMS = {
    'ExternParamsFile': '',
    'UseExternParamsFile': '0',
    'ImpulseGeneratorSource': '1',
    'TriggerType': '0',
    'TriggerSignalMode': '0',
    'PhotocellSource': '0',
    'Resolution': '550',
    'PrintDistance': '0',
    'TxtRepetitions': '1',
    'RepetitionDistance': '0',
    'RowMultiply': '0',
    'UpsideDownPrint': '0',
    'ReversePrint': '0',
    'ShaftDirection': '0',
    'TextHeight': '0',
    'CleaningRows': '0',
    'Pressure': '35',
    'DotSize': '3',
}

BARCODE_OBJECT = {
    'ObjectType': 'BarcodeObject',
    'ObjectName': 'Barcode 1',
    'x': '15',
    'y': '7',
    'w': '21',
    'h': '21',
    'AutoSize': '1',
    'Transparent': '0',
    'TransformMode': '0',
    'ObjectRotate': '0',
    'MustEdit': '0',
    'Printable': '1',
    'BarcodeID': 'Ex:58',
    'BorderType': '0',
    'BorderSize': '1',
    'Signature': '0',
    'SubType': '-1',
    'AutoAdjustText': '0',
    'FontName': 'fonts/User/MTCORSVA.ttf',
    'FontSize': '20',
    'FontSizeY': '20',
    'FontBold': '0',
    'FontItalic': '0',
    'Cmd': 'zint --output=%O --scale=0.5 --barcode=58 -d%T',
}

EDITOR_DATA_FIELDS = {f'Field{i}': '' for i in range(14)}
EDITOR_DATA_FIELDS['Field14'] = '1'


def generate_prj(text, output_path=None):
    """Build an EBS .prj file whose barcode object encodes `text`.

    Returns the path written to.
    """
    root = et.Element('EBS_PrinterProject')
    et.SubElement(root, 'ProjectSettings', PROJECT_SETTINGS)
    et.SubElement(root, 'PrintingParams', PRINTING_PARAMS)

    obj = et.SubElement(root, 'Object', {**BARCODE_OBJECT, 'Text': text})
    et.SubElement(obj, 'EditorData', EDITOR_DATA_FIELDS)

    xml_str = minidom.parseString(et.tostring(root)).toprettyxml(indent='  ')
    xml_str = '\n'.join(line for line in xml_str.splitlines() if line.strip())

    if output_path is None:
        output_path = f'{text}.prj'

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(xml_str)

    return output_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('text', help='text to encode in the barcode object')
    parser.add_argument('-o', '--output', help='output .prj path (default: <text>.prj)')
    args = parser.parse_args()

    path = generate_prj(args.text, args.output)
    print(f'Wrote {path}')


if __name__ == '__main__':
    main()
