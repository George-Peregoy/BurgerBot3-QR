import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from ebs_printer_interfaces.action import PrintQR

from ebs_printer.http_client import EBSHttpClient, EBSError
from ebs_printer.prj_gen import generate_prj
from ebs_printer.exp_build import build_exp
from ebs_printer_node import printer_config

import os
import tempfile
import requests
import time

class EBSPrinterActionServer(Node):
    def __init__(self):
        super().__init__('ebs_printer_node')

        # Log in to printer using predefined config (printer_config.py)
        self.client = EBSHttpClient(
            ip=printer_config.IP,
            user_id=printer_config.USER_ID,
            password=printer_config.PASSWORD,
        )

        self.client.login()   # persistent session

        self._action_server = ActionServer(
            self, PrintQR, 'print_qr', 
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)    # publish movement command to bot for printing

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT    # allows cancels mid execution (can't touch self.client here)

    def execute_callback(self, goal_handle):
        feedback = PrintQR.Feedback()
        result = PrintQR.Result()
        text = goal_handle.request.text
        outdir = tempfile.mkdtemp(prefix='ebs_')    # fresh temp dir per goal 

        def stage(name):
            feedback.stage = name
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(name)
        
        try:
            stage('GENERATING')
            name = f'path_{int(time.time())}.prj'   # unique name to avoid overwriting 
            prj_path = generate_prj(text, os.path.join(outdir, name))
            # change above filepath/filename to f'path_{env_number}.prj' for unique prj names 

            stage('PACKAGING')
            exp_path = build_exp(prj_path)   # generates fresh path.exp based on new path.prj

            stage('UPLOADING')
            self.client.upload(exp_path)

            stage('SELECTING')
            project_path = '/' + os.path.basename(prj_path)
            self.client.select(project_path)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Cancelled before printing'
                return result

            stage('ARMING')
            self.client.start()     # arm the printer for printing

            stage('PRINTING')
            self._drive_print_stroke(goal_handle)   # roll forward to print QR code
            # self.client.print_project('/' + os.path.basename(prj_path), duration=2.0) # -> print /path.prj

            stage('STOPPING')
            self.client.stop()      # end printing mode

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()  # mark goal handle as canceled
                result.success = False
                result.message = 'Print cancelled'
                return result
            
            goal_handle.succeed()
            result.success = True
            result.message = 'OK'

        except (EBSError, requests.exceptions.RequestException) as e:
            try:
                self.client.stop()  # disarm on error
            except Exception:
                pass
            self.get_logger().error(f'Print job failed: {e}')
            goal_handle.abort()
            result.success = False
            result.message = str(e)
        
        return result

    def _drive_print_stroke(self, goal_handle):
        twist = Twist()
        twist.linear.x = printer_config.PRINT_SPEED     # speed in m/s
        end = time.time() + printer_config.PRINT_STROKE_SEC
        try:
            while time.time() < end:
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('Stroke cancelled')
                    break
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)    # ~ 20 Hz
        finally:
            self.cmd_vel_pub.publish(Twist())   # safeguard to make sure robot stops
    
    def destroy_node(self):
        try:
            self.client.stop() # Don't leave printer armed in case node dies early
        except Exception:
            pass
        super().destroy_node()

        

def main(args=None):
    rclpy.init(args=args)
    node = EBSPrinterActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()