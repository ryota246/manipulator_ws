import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float32MultiArray
from custom_msgs.msg import BoundingBox
from custom_msgs.msg import Pos
from custom_msgs.action import Approach
import math
import numpy as np
import time
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time

#パラメーター
NO_BOX_COUNT_LIMIT = 10
TARGET_Y = 0.0
IMAGE_SIZE = [480,640]
X_GAIN = [0.0,0.0,0.0]
Y_GAIN = [0.0,0.0,0.0]
SCALING = 1.0
APPROACH_Z = 100.0
SPEED_LIMIT = 10.0
X_TOLERANCE = 5
Y_TOLERANCE = 5



class approach(Node):
    def __init__(self):
        super().__init__('approach_node')
        self.sub = self.create_subscription(BoundingBox, 'bounding_box', self.bounding_box_callback, 10)
        self.pub = self.create_publisher(Pos, 'pos', 10)
        self.action_server = ActionServer(self, Approach, 'approach', self.execute_callback,goal_callback=self.goal_callback, cancel_callback=self.cancel_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.box_data = BoundingBox()
        self.pos_data = Pos()
        self.pos_data.x = 100
        self.pos_data.y = 0.0
        self.pos_data.z = 100
        self.pos_data.yaw = 0.0
        self.pos_data.grip = 0.0
        self.run = False
        self.finish_event = threading.Event()
        self.lock = threading.Lock()


    def bounding_box_callback(self, msg):
        with self.lock:
            self.box_data = msg

    def timer_callback(self):###################################メインループ#################################


        if self.run == False:
            return
        
        
        with self.lock:
            if self.box_data == None:
                self.get_logger().info("No bounding box data received yet")
                return
            box_data = self.box_data

        if len(box_data.xyxy) % 4 != 0:
            self.get_logger().info("Invalid bounding box data received")
            return
            

        if box_data.has_box == False:
            self.get_logger().info("No bounding box detected count: {}".format(self.no_box_count))
            self.no_box_count += 1
            if self.no_box_count >= NO_BOX_COUNT_LIMIT:
                self.get_logger().info("No bounding box detected for a while, stopping approach")
                self.run = False
                self.has_box = False
                self.finish_event.set()
            return
        self.no_box_count = 0

        #バウンディンぐボックスが一番左にあるものを選定
        min_index = 0
        for i in range(len(box_data.xyxy)//4):
            if i == 0:
                min_x = (box_data.xyxy[i*4] + box_data.xyxy[i*4+2]) / 2
            else:
                if (box_data.xyxy[i*4] + box_data.xyxy[i*4+2]) / 2 < min_x:
                    min_x = (box_data.xyxy[i*4] + box_data.xyxy[i*4+2]) / 2
                    min_index = i



        if self.state == "approach":
            self.get_logger().info("Approaching target")
            #バウンディングボックスの中心座標を計算
            center_x = (box_data.xyxy[min_index*4] + box_data.xyxy[min_index*4+2]) / 2
            center_y = (box_data.xyxy[min_index*4+1] + box_data.xyxy[min_index*4+3]) / 2



            target_x = IMAGE_SIZE[1] / 2
            error = [0,0]
            error[0] = TARGET_Y - center_y
            error[1] = center_x - target_x

            self.get_logger().info("Approach Error: {}".format(error))

            


            P_GAIN = [X_GAIN[0]*SCALING,Y_GAIN[0]*SCALING]
            I_GAIN = [X_GAIN[1]*SCALING,Y_GAIN[1]*SCALING]
            D_GAIN = [X_GAIN[2]*SCALING,Y_GAIN[2]*SCALING]

            for i in range(2):
                if self.first_loop == True:
                    self.prev_error = [0,0]
                    self.integral = [0,0]
                    self.first_loop = False
                self.integral[i] += error[i] * 0.1
                derivative = (error[i] - self.prev_error[i]) / 0.1
                output = P_GAIN[i] * error[i] + I_GAIN[i] * self.integral[i] + D_GAIN[i] * derivative
                if i == 0:
                    self.pos_data.y += output
                else:
                    self.pos_data.x += output
                self.prev_error[i] = error[i]

            if self.pos_data.x > SPEED_LIMIT:
                self.pos_data.x = SPEED_LIMIT
            if self.pos_data.x < -SPEED_LIMIT:
                self.pos_data.x = -SPEED_LIMIT
            if self.pos_data.y > SPEED_LIMIT:
                self.pos_data.y = SPEED_LIMIT
            if self.pos_data.y < -SPEED_LIMIT:
                self.pos_data.y = -SPEED_LIMIT
            
                pose_msg = Pos()
                pose_msg.x = self.pos_data.x
                pose_msg.y = self.pos_data.y
                pose_msg.z = APPROACH_Z
                pose_msg.yaw = 0
            
            if abs(error[0]) < Y_TOLERANCE and abs(error[1]) < X_TOLERANCE:#ゴール判定
                self.goal_count += 1
                if self.goal_count >= 5:
                    self.get_logger().info("Target reached, stopping approach")
                    self.state = "turn"
                    goal_x = self.pos_data.x
                    goal_y = self.pos_data.y
                    self.has_box = True
                    return
                


            
            self.first_loop = False


        if self.state == "turn":
            self.get_logger().info("Turning to target")
            
            angle = box_data.angle[min_index]
            self.pos_data.yaw = angle
            self.pos_data.x = goal_x
            self.pos_data.y = goal_y
            self.pos_data.z = APPROACH_Z

        self.pub.publish(self.pos_data)
        

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return rclpy.action.GoalResponse.ACCEPT
        pass

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received request to cancel goal')
        return rclpy.action.CancelResponse.ACCEPT
        

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')

        self.first_loop = True
        self.start_time = time.time()

        self.no_box_count = 0
        self.goal_count = 0
        self.state = "approach"




        self.run = True
        self.finish_event.clear()
        self.finish_event.wait()
        goal_handle.succeed()
        result = Approach.Result()
        if self.has_box == True:
            result.success = True
            result.goal_x = self.pos_data.x
            result.goal_y = self.pos_data.y
            result.goal_z = self.pos_data.z
            result.goal_yaw = self.pos_data.yaw
        else:
            result.success = False
        return result
        

def main(args=None):
    rclpy.init(args=args)
    approach_node = approach()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(approach_node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        approach_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()