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
import math
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup
)

#パラメーター
NO_BOX_COUNT_LIMIT = 30
TARGET_Y = 480 - 200
IMAGE_SIZE = [480,640]
X_GAIN = [1.5,0.0,0.0]
Y_GAIN = [1.5,0.0,0.0]
SCALING = 0.001
APPROACH_Z = 100.0
SPEED_LIMIT = 10000000
X_TOLERANCE = 10
Y_TOLERANCE = 10



class approach(Node):
    def __init__(self):
        super().__init__('approach_node')
        self.declare_parameter("debug_mode",0)
        self.debug_mode = self.get_parameter("debug_mode").value
        self.timer_group = ReentrantCallbackGroup()
        self.action_group = MutuallyExclusiveCallbackGroup()

        self.sub = self.create_subscription(BoundingBox, 'bounding_box', self.bounding_box_callback, 10, callback_group=self.timer_group)
        self.pub = self.create_publisher(Pos, 'pose_data', 10, callback_group=self.timer_group)
        self.action_server = ActionServer(self, Approach, 'approach', self.execute_callback,goal_callback=self.goal_callback, cancel_callback=self.cancel_callback, callback_group=self.action_group)
        self.timer = self.create_timer(0.01, self.timer_callback, callback_group=self.timer_group)
        self.box_data = BoundingBox()
        self.pos_data = Pos()
        
        self.finish_event = threading.Event()
        self.lock = threading.Lock()
        self.debug_img = np.zeros((IMAGE_SIZE[0], IMAGE_SIZE[1], 3), dtype=np.uint8)

        self.no_box_count = 0
        self.goal_count = 0
        self.state = "approach"
        self.first_loop = True
        self.run = False

        self.get_logger().info("Approach Node has been started.")


    def bounding_box_callback(self, msg):
        with self.lock:
            self.box_data = msg
        self.get_logger().info("Bounding box data received")

    def timer_callback(self):###################################メインループ#################################


        if self.run == False:
            return
        
        box_data = None
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
            time.sleep(0.3)
            return
        else:
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


        if self.no_box_count == 0:
            if self.state == "approach":
                self.get_logger().info("Approaching target")
                #バウンディングボックスの中心座標を計算
                center_x = (box_data.xyxy[min_index*4] + box_data.xyxy[min_index*4+2]) / 2
                center_y = (box_data.xyxy[min_index*4+1] + box_data.xyxy[min_index*4+3]) / 2



                target_x = (IMAGE_SIZE[1] / 2) - 80
                error = [0,0]
                error[0] = TARGET_Y - center_y
                error[1] = target_x - center_x

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

                    if output > SPEED_LIMIT:
                        output = SPEED_LIMIT
                    if output < -SPEED_LIMIT:
                        output = -SPEED_LIMIT
                    if i == 0:
                        self.pos_data.x += output
                    else:
                        self.pos_data.y += output
                    self.prev_error[i] = error[i]

                
                    pose_msg = Pos()
                    pose_msg.x = self.pos_data.x
                    pose_msg.y = self.pos_data.y
                    pose_msg.z = APPROACH_Z
                    pose_msg.yaw = 0.0
                    pose_msg.grip = "open"
                    pose_msg.mode = 1
                
                if abs(error[0]) < Y_TOLERANCE and abs(error[1]) < X_TOLERANCE:#ゴール判定
                    self.goal_count += 1
                    if self.goal_count >= 5:
                        self.get_logger().info("Target reached, stopping approach")
                        self.state = "turn"
                        self.has_box = True
                        self.label = box_data.labels[min_index]
                        return
                    


                
                self.first_loop = False


            if self.state == "turn":
                self.get_logger().info("Turning to target")
                
                angle = box_data.angles[min_index]
                now_angle = -(math.atan2(self.pos_data.y, self.pos_data.x) * 180 / math.pi)
                self.pos_data.yaw = now_angle + angle
                pose_msg = Pos()
                pose_msg.x = self.pos_data.x
                pose_msg.y = self.pos_data.y
                pose_msg.z = APPROACH_Z
                pose_msg.yaw = self.pos_data.yaw
                pose_msg.grip = "open"
                pose_msg.mode = 0

                self.run = False
                self.finish_event.set()

                

            self.pub.publish(self.pos_data)

            if self.debug_mode == 1:#デバッグ表示
                # 黒い画像を作成
                img = np.zeros((IMAGE_SIZE[0], IMAGE_SIZE[1], 3), dtype=np.uint8)

                # 全ての検出ボックスを緑で描画
                for i in range(len(box_data.xyxy)//4):
                    x1 = int(box_data.xyxy[i*4])
                    y1 = int(box_data.xyxy[i*4+1])
                    x2 = int(box_data.xyxy[i*4+2])
                    y2 = int(box_data.xyxy[i*4+3])

                    cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)

                # 制御対象を赤で描画
                x1 = int(box_data.xyxy[min_index*4])
                y1 = int(box_data.xyxy[min_index*4+1])
                x2 = int(box_data.xyxy[min_index*4+2])
                y2 = int(box_data.xyxy[min_index*4+3])

                cv2.rectangle(img, (x1, y1), (x2, y2), (0,0,255), 3)

                # 中心点
                cv2.circle(img, (int(center_x), int(center_y)), 5, (255,0,0), -1)

                # 目標位置
                target_x = IMAGE_SIZE[1] // 2
                target_y = int(TARGET_Y)

                cv2.circle(img, (target_x, target_y), 5, (0,255,255), -1)

                # 誤差ベクトル
                cv2.line(img,
                        (target_x, target_y),
                        (int(center_x), int(center_y)),
                        (255,255,0), 2)

                # エラー表示
                cv2.putText(img,
                            f"ErrorX : {error[1]:.1f}",
                            (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255,255,255),
                            2)

                cv2.putText(img,
                            f"ErrorY : {error[0]:.1f}",
                            (10,60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255,255,255),
                            2)

                cv2.putText(img,
                            f"State : {self.state}",
                            (10,90),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            (255,255,255),
                            2)

                cv2.imshow("Approach Debug", img)
                cv2.waitKey(1)
        

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
        self.pos_data.x = 210.0
        self.pos_data.y = 40.0
        self.pos_data.z = 40.0
        self.pos_data.yaw = 0.0
        self.pos_data.grip = "open"
        




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
            result.label = self.label
        else:
            result.label = "none"
            result.success = True

        self.get_logger().info(result.label)
            
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