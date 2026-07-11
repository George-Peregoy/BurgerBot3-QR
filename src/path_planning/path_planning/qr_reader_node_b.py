import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from pyzbar.pyzbar import decode

class QRPublisher(Node):

    def __init__(self):
        super().__init__('qr_publisher_node_b')

        self.publisher_ = self.create_publisher(String, 'qr_data', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2) # capture from /dev/video0
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320) # backend V4L2
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_FPS, 5)

        self.qr_string = None
        self.qr_found = False

    def timer_callback(self):

        if self.qr_found:
            return

        ret, frame = self.cap.read()
        self.get_logger().info(f"READ QR: {ret}")
        
        if ret:
            qr_data = decode(frame)
            if qr_data:
                qr_string = qr_data[0].data.decode('utf-8')
                msg = String()
                msg.data = qr_string
                self.get_logger().info(f"Publishing QR data: {qr_string}")
                self.publisher_.publish(msg)
                self.qr_found = True

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    qr_publisher = QRPublisher()
    rclpy.spin(qr_publisher)
    qr_publisher.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()