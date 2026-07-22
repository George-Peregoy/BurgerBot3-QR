import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

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

        self._action_server = ActionServer(self, PrintQR, 'print_qr', execute_callback=self.execute_callback,)

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

            stage('PRINTING')
            self.client.print_project('/' + os.path.basename(prj_path), duration=2.0)   # -> print /path.prj
            
            goal_handle.succeed()
            result.success = True
            result.message = 'OK'
        except (EBSError, requests.exceptions.RequestException) as e:
            self.get_logger().error(f'Print job failed: {e}')
            goal_handle.abort()
            result.success = False
            result.message = str(e)
        
        return result
    
    def destroy_node(self):
        try:
            self.client.stop() # Don't leave printer armed in case node dies early
        except Exception:
            pass
        super().destroy_node()

        

def main(args=None):
    rclpy.init(args=args)
    node = EBSPrinterActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()