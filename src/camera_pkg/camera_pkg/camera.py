import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class camera_node(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.declare_parameter("debug_mode",False)
        self.debug_mode = self.get_parameter("debug_mode").value

        if self.debug_mode:
            self.get_logger().info("debug mode")

        self.pub = self.create_publisher(Image,'/camera_image',10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.01,self.timer_callback)

        self.cap = cv2.VideoCapture(2)

        self.get_logger().info("camera_node is up")

    
    def timer_callback(self):
        ret, frame =  self.cap.read()

        if not ret:
            self.get_logger().info("Can't receive frame. Exiting...")

        frame = cv2.resize(frame,(640,480))
        #frame = cv2.flip(frame, -1)







        gamma = 1

        table = np.array([
            ((i / 255.0) ** gamma) * 255
            for i in np.arange(256)
        ]).astype("uint8")

        frame = cv2.LUT(frame, table)



        # lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        # l, a, b = cv2.split(lab)

        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        # l = clahe.apply(l)

        # lab = cv2.merge((l,a,b))
        # frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)






        msg = self.bridge.cv2_to_imgmsg(frame,encoding="bgr8")

        self.pub.publish(msg)
        
        if self.debug_mode:
            cv2.imshow('Camera image',frame)
            cv2.waitKey(1)
            


        

            
        

def main():
    rclpy.init()

    node = camera_node()


    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


