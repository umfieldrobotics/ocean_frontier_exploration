import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header
import numpy as np
from sensor_msgs_py import point_cloud2
import cv2
from cv_bridge import CvBridge


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # underwater image 
        self.uwimgage = self.create_subscription(
            Image,
            '/Oceansim/rgb',
            self.uw_image_callback,
            10)
        self.edge_pub = self.create_publisher(
            Image,
            '/Oceansim/edges',
            10)


        self.subscription = self.create_subscription(
            Image,
            '/Oceansim/sonar_image',
            self.listener_callback,
            10)
        self.subscription
        
        self.pub = self.create_publisher(
            PointCloud2,
            "/sonar/occupancy",
            10
        )
        self.intens_thresh = 50
        self.image = None
        self.bridge = CvBridge()

    def uw_image_callback(self, msg):
        self.image = msg

        cvimv = self.bridge.imgmsg_to_cv2(self.image, desired_encoding='passthrough')
        grayscale = cv2.cvtColor(cvimv, cv2.COLOR_RGBA2GRAY)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced  = clahe.apply(grayscale)

        canny_Edge = cv2.Canny(enhanced, 100,200)

        #TODO cont. here
        print("running")
        rosimg = self.bridge.cv2_to_imgmsg(canny_Edge)
        self.edge_pub.publish(rosimg)
        # segment image

    def listener_callback(self, msg):
        sonar_read = np.frombuffer(msg.data,dtype=np.uint8)
        sonar_read = sonar_read.reshape((msg.height, msg.width,4))[:, :, 0]
        height, width = msg.height, msg.width
        rows, cols = np.where(sonar_read > self.intens_thresh)
        thresholded = sonar_read[rows, cols]

        distance = 0.2 + (rows /height) * (3.0 - 0.2)
        fov = np.deg2rad(130)
        theta = -(fov /2) + (cols / width) * (fov/2)

        #cartesian coordinates
        x = distance * np.cos(theta)
        y = distance * np.sin(theta)
        z = np.zeros_like(x)

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = "sonar"
        points = point_cloud2.create_cloud_xyz32(header,np.column_stack((x,y,z)))
        self.pub.publish(points)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()