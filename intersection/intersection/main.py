import cv2
import numpy as np
import matplotlib.pyplot as plt
import poly_point_isect
import math
from line_isect import IntersectionPoints
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist,Pose,PointStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Bool
from sklearn.cluster import DBSCAN
import numpy as np
import math
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
import numpy as np
from scipy import optimize
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import Image
import tf2_ros
import tf2_geometry_msgs
import struct
import tf_transformations

class IntersectionPath(Node):
    def __init__(self):
        self.inter_class = None
        self.inter_class2 = None    
        self.lane_image = None
        self.result_image = None

        self.navigator = BasicNavigator()
        self.object_subscriber_flag = self.create_publisher(Bool, '/object_flag', self.object_callback, 10)
        self.horizontal_line_subscriber_flag = self.create_publisher(Bool, '/hor_flag', self.horizontal_line_callback, 10)
        self.subscription=self.create_subscription(Image, '/lanes', self.lanes_callback,10)
        self.path_publisher = self.create_publisher(Path, 'intersection_path', 10)

    def object_callback(self, msg):
        if msg == True:
            self.object_ahead = True

        if msg == True and self.horizontal_line_ahead == True:
            self.intersection_ahead = True

    def horizontal_line_callback(self, msg):
        if msg == True:
            self.horizontal_line_ahead = True
        
        if msg == True and self.object_ahead == True:
            self.intersection_ahead = True

    def lanes_callback(self, msg):
        self.lane_image = msg

        if self.intersection_ahead == True:
            self.plan_path()

    def image_to_world(self, x, y):
        return x, y
    
    def plan_path(self):
        del self.inter_class, self.inter_class2

        self.result_image = np.copy(self.lane_image)

        self.inter_class = IntersectionPoints(img = self.lane_image, result_image=self.result_image, ver_ratio=1.7, hor_ratio=2)
        self.inter_class2 = IntersectionPoints(img = self.lane_image, result_image=self.result_image, ver_ratio=1.7, hor_ratio=2)

        start_point = self.inter_class2.get_lines(cond = 'bottom')
        goal_point = self.inter_class.get_lines(cond = 'right')

        diag_center = (goal_point + start_point)/2
        b = np.linalg.norm(goal_point-start_point)
        radius = b/np.sqrt(2)

        theta1 = -(np.arctan((goal_point[1]-start_point[1])/(goal_point[0]-start_point[0])))
        center = diag_center[0] + (b/2)*np.sin(theta1), diag_center[1] + (b/2)*np.cos(theta1)
        self.result_image[int(center[1]), int(center[0])] = [0, 255, 0]

        # Calculate the radius using the distance to one of the points
        radius = np.linalg.norm(start_point - center)  # Distance from center to point1

        # Number of points to draw along the circle
        number_of_points = 500

        init_angle = np.arctan((start_point[1]-center[1])/(start_point[0]-center[0]))
        fin_angle = np.arctan((goal_point[1]-center[1])/(goal_point[0]-center[0]))

        if (start_point[1]-center[1] < 0) and (start_point[0]-center[0] < 0):
            init_angle = np.pi + np.arctan((start_point[1]-center[1])/(start_point[0]-center[0]))
        elif (start_point[1]-center[1] >= 0) and (start_point[0]-center[0] >= 0):
            init_angle = np.arctan((start_point[1]-center[1])/(start_point[0]-center[0]))


        if (goal_point[1]-center[1] < 0) and (goal_point[0]-center[0] < 0):
            fin_angle = np.pi + np.arctan((goal_point[1]-center[1])/(goal_point[0]-center[0]))
        elif (goal_point[1]-center[1] >= 0) and (goal_point[0]-center[0] >= 0):
            fin_angle = np.arctan((goal_point[1]-center[1])/(goal_point[0]-center[0]))


        # Generate points on the circle
        angles = np.linspace(init_angle + np.pi/16, np.pi - fin_angle + 2*np.pi/16, number_of_points)  # Full circle
        circle_path = []

        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        path_poses = []

        for angle in angles:
            x = center[0] + radius * np.cos(angle)
            y = center[1] + radius * np.sin(angle)
            z = 0.0

            x += 50
            y += 50

            circle_path.append((x, y))

            x, y = self.image_to_world(x, y)

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            roll = 0.0
            pitch = 0.0
            yaw = np.round(math.atan((math.sin(angle)-math.sin(angle-0.01))/(math.cos(angle)-math.cos(angle-0.01))))
            qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
            pose.pose.orientation.w = qw
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            path_msg.poses.append(pose)
            path_poses.append(pose)
        
        # Draw the circle on the image
        for point in circle_path:
            # Make sure the points are within image bounds
            if 0 <= int(point[1]) < self.result_image.shape[0] and 0 <= int(point[0]) < self.result_image.shape[1]:
                # Draw the circle point (set to green)
                self.result_image[int(point[1]), int(point[0])] = [0, 255, 0]  # [B, G, R] format

        # Display the result_image with the drawn arc
        # cv2.imshow('Elliptical Arc', self.result_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        cv2.imwrite('line_parking.png', self.result_image)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Published a new trajectory.')
        # self.navigator.followPath(path_msg)
        self.navigator.goThroughPoses(path_poses)

        while not self.navigator.isTaskComplete():
            print("navigating")
        

def main(args=None):
    rclpy.init(args=args)
    node = IntersectionPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


###############################################################################################################################################
# Number of points to draw along the arc
# number_of_points = 500

# Calculate the center of the circle (intersection of the major and minor axes)
# We assume the center is the same distance from both the start and goal points
# center = np.array([start_point[0], goal_point[1]])

# # Radius of the circle (distance from center to start_point)
# radius = np.linalg.norm(start_point - center)

# # Parametric angles for the arc (from 0 to 90 degrees for quarter circle)
# angles = np.linspace(np.pi, 3/2*np.pi/2, number_of_points)  # Adjust this range for different arcs

# # Path for the arc
# path = []

# for angle in angles:
#     # Parametric equation of the circle
#     x = center[0] + radius * np.cos(angle)
#     y = center[1] + radius * np.sin(angle)
    
#     path.append((x, y))

# # Draw the arc on the image
# for point in path:
#     # Make sure the points are within image bounds
#     if 0 <= int(point[1]) < result_image.shape[0] and 0 <= int(point[0]) < result_image.shape[1]:
#         # Draw the arc point (set to green)
#         result_image[int(point[0]), int(point[1])] = [0, 255, 0]  # [B, G, R] format
####################################################################################################################################################