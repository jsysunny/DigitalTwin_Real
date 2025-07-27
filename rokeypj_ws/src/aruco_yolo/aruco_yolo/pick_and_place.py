#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Frame Capture and JPEG Compression

This script captures frames from a webcam using OpenCV,
retrieves the raw image's width and height, compresses
the image to JPEG format, and also determines the size
of the decoded (compressed) image.

Author: Rujin Kim
Date: 2025-05-17
"""
import rclpy
from rclpy.node import Node
from aruco_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import getkey
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from geometry_msgs.msg import Twist, Pose, PoseArray
from turtlebot_cosmo_interface.srv import MoveitControl
from aruco_yolo.moveit_client import TurtlebotArmClient
import time
import ast


# ANSI 색상 코드 정의
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"  # 색상 초기화


class ArucoMarkerListener(Node):
    def __init__(self):
        super().__init__('aruco_marker_listener')

        # Change this to the desired marker ID from pick_n_place.launch.py file, Declare parameter with default integer value
        self.markerid = self.declare_parameter('markerid', 1).get_parameter_value().integer_value

        self.target_marker_id = self.markerid 
        """
        self.subscription = self.create_subscription(
            MarkerArray,
            'detected_markers',
            self.aruco_listener_callback,
            10)
        """
        self.pick_sub = self.create_subscription(
            MarkerArray,
            '/pick_trigger',
            self.aruco_listener_callback,
            10
        )
        #self.subscription  # prevent unused variable warning
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 2)

        self.twist = Twist()
        self.finish_move = False


        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_joint = False
        self.marker = []
        """---------------------------------"""

        #sample_pkg/src/simple_manager_node.py
        # 상태 변수
        self.aruco_marker_found = False
        self.task_completed = False
        self.armrun = False
        self.aruco_location_x = 0
        self.aruco_location_y = 0
        self.aruco_location_z = 0       

        self.marker_id = None
        self.state = 'START'  

        self.count = 0
        self.aruco_pose = None  # Aruco marker의 pose 정보를 저장할 변수

        #self.create_timer(1.0, self.run_tasks)
        
        #self.twist = Twist()

    def pick_callback(self, marker_array_msg: MarkerArray):
        if not marker_array_msg.markers:
            self.get_logger().warn("Received empty pick_trigger message.")
            return

        marker_msg = marker_array_msg.markers[0]  # 첫 번째 마커 사용
        self.get_logger().info(f"Received pick trigger with marker ID {marker_msg.id}")
        
        # 좌표 저장
        self.aruco_location_x = marker_msg.pose.pose.position.x
        self.aruco_location_y = marker_msg.pose.pose.position.y
        self.aruco_location_z = marker_msg.pose.pose.position.z
        
        # 픽 동작 수행
        self.aruco_arm_controll() 



    def joint_states_callback(self, msg):

        print("joint_states_callback")

        if self.get_joint == False:
            return
        
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint1' in name:
                print(f'joint1 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint2' in name:
                print(f'joint2 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint3' in name:
                print(f'joint3 : {position}')
        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            if 'joint4' in name:
                print(f'joint4 : {position}')

    def aruco_listener_callback(self, msg):
        print("aruco_listener_callback")
        for marker in msg.markers:
            if marker.id == self.target_marker_id:
                self.get_logger().debug(f'Marker ID: {marker.id}, PositionZ: {marker.pose.pose.position.z}')
                self.get_logger().info(f'Position: x:[{marker.pose.pose.position.x}, y:[{marker.pose.pose.position.y},z:[{marker.pose.pose.position.z}]] ')
                self.get_logger().info(f'Orientation: x:[{marker.pose.pose.orientation.x}, y:[{marker.pose.pose.orientation.y},z:[{marker.pose.pose.orientation.z}]] ')

                self.aruco_position_x = marker.pose.pose.position.x
                self.aruco_position_y = marker.pose.pose.position.y               
                self.aruco_position_z = marker.pose.pose.position.z
                """
                # if marker.pose.pose.position.z > 0.30:
                if marker.pose.pose.position.z > 0.41:
                    self.get_logger().info(f'publish_cmd_vel(0.10)')
                    self.publish_cmd_vel(0.10)
                elif marker.pose.pose.position.z > 0.31:
                    self.get_logger().info(f'publish_cmd_vel(0.06)')                    
                    self.publish_cmd_vel(0.06)
                elif marker.pose.pose.position.z > 0.21 :
                    self.get_logger().info(f'publish_cmd_vel(0.04)')                    
                    self.publish_cmd_vel(0.04)
                else:
                    self.publish_cmd_vel(0.0)
                    self.get_logger().info(f'finish_move = True')                    
                    self.finish_move = True
                """
                self.get_logger().info(f'aruco_arm_controll = True')
                self.aruco_arm_controll()

                break

    def publish_cmd_vel(self, linear_x):
        self.twist.linear.x = linear_x
        self.twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(self.twist)  


##################################################################################            
##################################################################################
    def append_pose_init(self, x,y,z):
        pose_array = PoseArray()
        pose = Pose()

        pose.position.x = x
        pose.position.y =  y
        pose.position.z =  z
        pose_array.poses.append(pose)
        
        self.get_logger().info(f"{CYAN}Pose initialized - x: {x}, y: {y}, z: {z}{RESET}")

        return pose_array

    def aruco_arm_controll(self):
        arm_client = TurtlebotArmClient()

        print("Impossible Mission Start")

        print(f"Mission Aruco marker Locaion coordinates: {self.aruco_position_x}, {self.aruco_position_y}, {self.aruco_position_z}")

        ##### TEST, if I cound aruco_marker...
        self.aruco_marker_found = True

        if self.aruco_marker_found:
            self.armrun = True
            """
            self.aruco_location_x = -0.92
            self.aruco_location_y = -0.45
            self.aruco_location_z = 0.33
            """
            #print(f"Remove Rock Initial Position")
            #self.point.positions = [0.0, -0.058, -0.258, 1.94]
            #print("point",self.point.positions)
            #self.joint_pub.publish(self.trajectory_msg)


            print("Move forward to Rock")
        
            #self.twist.linear.x = 2
            #self.twist.angular.z = 0.0
            #self.cmd_vel_publisher.publish(self.twist)         

            #self.cmd_vel_publisher.publish(Twist())
            
            print("Home Position")
        
            response = arm_client.send_request(1, "01_home1")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)
            
            print("Gripper Open")

            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            
            print("Box Front")
            response = arm_client.send_request(1, "02_box_front")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            print("Move to Box")
            response = arm_client.send_request(1, "03_move_to_box")
            arm_client.get_logger().info(f'Response: {response.response}')           
            time.sleep(1)
            
            
            print("Cube Position Start")
            pose_array = self.append_pose_init(0.137496 - self.aruco_location_y + 0.05,0.00 - self.aruco_location_x ,0.122354 )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')

            pose_array = self.append_pose_init(0.137496 - self.aruco_location_y + 0.05,0.00 - self.aruco_location_x ,0.087354  )
            response = arm_client.send_request(0, "", pose_array)
            arm_client.get_logger().info(f'Response: {response.response}')     

            print("Gripper Close")
            response = arm_client.send_request(2, "close")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            print("Box Up")            
            response = arm_client.send_request(1, "04_box_up")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(1)

            print("Conveyor Up ")
            response = arm_client.send_request(1, "05_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(3)

            print("Conveyor Down ")
            response = arm_client.send_request(1, "06_conveyor_down")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(3)

            print("Gripper Open")
            response = arm_client.send_request(2, "open")
            arm_client.get_logger().info(f'Response: {response.response}')

            print("Conveyor Up ")
            response = arm_client.send_request(1, "07_conveyor_up")
            arm_client.get_logger().info(f'Response: {response.response}')
            time.sleep(3)

            print("Return Sweet Home ")
            response = arm_client.send_request(1, "08_home")
            arm_client.get_logger().info(f'Response: {response.response}')


            print("Impossible Mission Clear")

            #self.armrun = False
            #self.yolofind = False  # 작업 완료 후 초기화
            
            #self.count += 1
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerListener()
    #rclpy.spin(node)
    #rclpy.init(args=args)
    #node = ArucoMarkerListener()
    print("Waiting for /pick_trigger ...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    """
    joint_pub = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
    trajectory_msg = JointTrajectory()

    trajectory_msg.header = Header()
    trajectory_msg.header.frame_id = ''
    trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

    point = JointTrajectoryPoint()
    point.velocities = [0.0] * 4
    point.accelerations = [0.0] * 4
    point.time_from_start.sec = 0
    point.time_from_start.nanosec = 500


    # Initial Position
    #ros2 topic list => ros2 topic echo /joint_states --once
    #point.positions =  [0.0, 0.5236, -0.7156, 0.8203]  # (단위: 라디안) <= 0, -44, 9, 72
    #point.positions = [0.0, 0.7317, -0.6918, 0.7701]    
    point.positions = [0.0, 0.5317, -0.6918, 0.7701] 

    trajectory_msg.points = [point]
    joint_pub.publish(trajectory_msg)

    print("Pick and Place Init start...")
    rclpy.spin_once(node)

    while(rclpy.ok()):
        print("Key Input Waiting.")
        print("0: lane tracking position, move positions")
        print("1: cube home, move postions")
        print("2: box home, move postions")
        print("3: move to aroco_marker")
        print("4: forward moving, wheel move forward")
        print("5: backward moving, wheel move backward")   
        print("s: stop moving, wheel move backward")    
        print("8: get joint_states, Ex:joint1 : -8.528126114804024e-05는 소수점 이하 매우 작은 음수 값으로, 약 −0.00008528126114804024 라디안 (radians)을 의미합니다.")              
        print("a: pick and place, goal states")           

        key_value = getkey.getkey()
        if key_value == '0':          # lane tracking
            print(f"No. {key_value}")
            point.positions = [0.0, 0.5317, -0.6918, 0.7701] 
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)   

        elif key_value == '1':          # cube home
            print(f"No. {key_value}")
            point.positions = [0.0, -0.058, -0.258, 1.94]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == '2':        # box hom            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)      
            print(f"No. {key_value}")
            point.positions = [0.0, -1.052, 1.106, 0.029]
            print("point",point.positions)
            joint_pub.publish(trajectory_msg)

        elif key_value == '3':        # move wheel to aruco cube
            print(f"No. {key_value}")
            while True:
                rclpy.spin_once(node)
                if(node.finish_move == True):
                    node.finish_move = False
                    break
                
        elif key_value == '4':        # forward
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '5':        # backward
            print(f"No. {key_value}") 
            node.twist.linear.x =-0.5
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == 's':        # backward
            print(f"No. {key_value}") 
            node.twist.linear.x =0.0
            node.twist.angular.z = 0.0
            node.cmd_vel_publisher.publish(node.twist) 

        elif key_value == '6':        # forward + left turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z = 0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '7':        # forward + right turn
            print(f"No. {key_value}")
            node.twist.linear.x = 0.5
            node.twist.angular.z =-0.1
            node.cmd_vel_publisher.publish(node.twist)            

        elif key_value == '8':        # get joint value
            print(f"No. {key_value}")
            node.get_joint = True
            rclpy.spin_once(node)
  
        elif key_value == '9':        # distance and offset for marker
            print(f"No. {key_value}")
            rclpy.spin_once(node)
        elif key_value == 'a':
            print(f"No. {key_value}")
            node.aruco_arm_controll()
            
        elif key_value == 'q':
            print(f"No. {key_value}")
            break

    node.destroy_node()
    rclpy.shutdown()
    """

if __name__ == '__main__':
    main()

