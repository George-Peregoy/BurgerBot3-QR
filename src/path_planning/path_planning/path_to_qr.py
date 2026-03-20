import qrcode
from qrcode.exceptions import DataOverflowError
import zlib
import os

def path_to_qr(path: str, output_dir: str, env_number: int):
    """
    Encodes path into qr code.

    Parameters
    ----------
    path : str
        Flattened list of points as a str. Example "5 5 0"
    output_dir : str
        Path where QR codes are saved.
    """
    # Create a QR code instance
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_L,
        box_size=10,
        border=4,
    )
    
    compressed_data = zlib.compress(path.encode())

    try:
        # Add data to the QR code
        qr.add_data(compressed_data)
        qr.make(fit=False)

        # Create an image of the QR code
        img = qr.make_image(fill='black', back_color='white')

        # Save the image
        output_path = os.path.join(output_dir, f"qr_code_{env_number}.png")
        img.save(output_path)
        print(f"QR code generated and saved to {output_path}")

    except DataOverflowError:
        print("Error: The data is too large to fit in a Version 1 QR code.")

if __name__=="__main__":
    path_to_qr()