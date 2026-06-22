import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO
from sensor_msgs.msg import Image
from custom_msgs.msg import BoundingBox



class yolo_node(Node):

    def __init__(self):
        super().__init__('yolo_node')



        self.declare_parameter("debug_mode",0)
        self.debug_mode = self.get_parameter("debug_mode").value

        self.model = YOLO("/home/ryota/manipulator_pj/manipulator_ws/src/yolo_pkg/yolo_pkg/best.pt")

        if self.debug_mode:
            self.get_logger().info("debug mode")

        self.image_sub = self.create_subscription(Image,"/camera_image",self.image_callback,10)

        self.box_pub = self.create_publisher(BoundingBox,"/bounding_box",10)

        self.timer = self.create_timer(0.1,self.timer_callback)

        self.image =None
        self.bridge = CvBridge()

    def image_callback(self,msg):
        image = msg
        self.image = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        pass

    def timer_callback(self):

        if self.image is None:
            return

        msg = BoundingBox()
        
        results = self.model(self.image)
        result = results[0]

        box_points = []
        confidences = []
        labels = []
        angles = []

        
        gray_image = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)

        _ , binary = cv2.threshold(gray_image,160,255,cv2.THRESH_BINARY)

        if self.debug_mode == 1:
            debug_image = self.image.copy()
        if self.debug_mode == 2:
            debug_binary = binary.copy()


        for i , box in enumerate(result.boxes):
            xyxy = box.xyxy[0]
            
            for j in range(4):
                box_points.append(int(xyxy[j]))
            confidence = float(box.conf)
            confidences.append(confidence)
            class_id = int(box.cls[0])
            label = result.names[class_id]
            labels.append(label)


            x1 = int(xyxy[0])
            y1 = int(xyxy[1])
            x2 = int(xyxy[2])
            y2 = int(xyxy[3])

            binary_roi = binary[y1:y2,x1:x2]

            contours, _ = cv2.findContours(binary_roi,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                largest_contour = max(contours,key=cv2.contourArea)
                rect = cv2.minAreaRect(largest_contour)
                angle = rect[2]
                if angle < -45:
                    angle += 90
                angles.append(angle)

            else:
                angles.append(None)
                angle = 0







            

            
            if self.debug_mode == 1:

                
                cv2.rectangle(
                    debug_image,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                text = f"{label} {confidence:.2f} {angle}"

                cv2.putText(
                    debug_image,
                    text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )


            if self.debug_mode == 2:

                
                cv2.rectangle(
                    debug_binary,
                    (x1, y1),
                    (x2, y2),
                    (0, 0, 0),
                    2
                )

                text = f"{label} {confidence:.2f} {angle}"

                cv2.putText(
                    debug_binary,
                    text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (125, 0, 0),
                    2
                )

        msg.xyxy = box_points
        msg.confidence = confidences
        msg.labels = labels

        self.box_pub.publish(msg)


        if self.debug_mode == 1:#デバッグ表示
            cv2.imshow('yolo_node',debug_image)
            cv2.waitKey(1)

        if self.debug_mode == 2:
            cv2.imshow('yolo_node',debug_binary)
            cv2.waitKey(1)
    



def main():
    rclpy.init()
    node = yolo_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()

