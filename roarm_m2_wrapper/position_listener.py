import cv2 as cv
import numpy as np
import math

from geometry_msgs.msg import Pose, PoseArray

import rclpy
from rclpy.node import Node
import tf_transformations

class Position_listener(Node):

    def __init__(self):
        super().__init__('position_listener')
        self.publisher_ = self.create_publisher(PoseArray, 'objects_position', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.cap = cv.VideoCapture(0)  # Open the video capture
        self.cx = 0
        self.cy = 0
        self.angle = 0

    def timer_callback(self):
        msg = PoseArray()

        self.capture_frame(msg)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

    def capture_frame(self, msg):
        ret, frame = self.cap.read()
        if not ret:
            return
        
        belt = frame[294:1840, 120:520]
        gray = cv.cvtColor(belt, cv.COLOR_BGR2GRAY)
        _, bw = cv.threshold(gray, 45, 255, cv.THRESH_BINARY_INV)
        
        cv.imshow('threshold', bw)
        
        contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        
        for c in contours:
            area = cv.contourArea(c)
            if area < 20 or area > 1000000000:
                continue
            
            rect = cv.minAreaRect(c)
            box = cv.boxPoints(rect)
            box = np.intp(box)

            center = (int(rect[0][0]), int(rect[0][1]))
            width = int(rect[1][0])
            height = int(rect[1][1])
            angle = int(rect[2])

            M = cv.moments(c)
            if M["m00"] != 0:
                self.cx = int(M["m10"] / M["m00"])
                self.cy = int(M["m01"] / M["m00"])
                cv.circle(belt, (self.cx, self.cy), 5, (255, 255, 0), -1)
                cv.putText(belt, f"Centroid ({self.cx}, {self.cy})", (self.cx - 60, self.cy - 20),
                           cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            
            if width < height:
                angle = 90 - angle
            else:
                angle = -angle

            self.angle = angle

            pose = Pose()
            pose.position.x = float(self.cx)
            pose.position.y = float(self.cy)
            pose.position.z = 0.0

            roll_deg = 0.0
            pitch_deg = 0.0
            yaw_deg = float(angle)

            # Convert angles from degrees to radians
            roll = math.radians(roll_deg)
            pitch = math.radians(pitch_deg)
            yaw = math.radians(yaw_deg)

            quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            # Add the pose to the PoseArray
            msg.poses.append(pose)

            cv.drawContours(belt, [box], 0, (0,0,255), 2)

        cv.imshow("Frame", frame)
        cv.imshow("Belt", belt)
        cv.waitKey(1)

    def destroy(self):
        self.cap.release()
        cv.destroyAllWindows()

def main(args=None):
    rclpy.init()
    position_listener = Position_listener()
    rclpy.spin(position_listener)
    position_listener.destroy_node()
    rclpy.shutdown()
    position_listener.destroy()

if __name__ == "__main__":
    main()
