
import rclpy
from rclpy.node import Node

from autoware_auto_perception_msgs.msg import PredictedObjects
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from nav_msgs.msg import OccupancyGrid

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('fake_perception')
        self.object_publisher_ = self.create_publisher(PredictedObjects, '/perception/object_recognition/objects', 10)
        self.segmentation_publisher_ = self.create_publisher(PointCloud2, '/perception/obstacle_segmentation/pointcloud', 10)
        self.occupancy_grid_publisher = self.create_publisher(OccupancyGrid, '/perception/occupancy_grid_map/map', 10)
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias', self.listener_callback, 10)
        timer_period = 1 / 10  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        fake_pointcloud = PointCloud2()
        fake_pointcloud.header.stamp = self.get_clock().now().to_msg()
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1
        self.segmentation_publisher_.publish(fake_pointcloud)

    def listener_callback(self, msg):
        fake_obstacles = PredictedObjects()
        fake_obstacles.header = msg.header
        #print(msg)

        converted_pose = PoseWithCovariance()
        converted_pose.pose = msg.pose.pose
        converted_pose.covariance = msg.pose.covariance
        fake_obstacles.current_pose_with_covariance = converted_pose
        self.object_publisher_.publish(fake_obstacles)

        fake_occupancy_grid = OccupancyGrid()
        fake_occupancy_grid.header = msg.header
        fake_occupancy_grid.info.resolution = 0.5
        fake_occupancy_grid.info.width = 200
        fake_occupancy_grid.info.height = 200
        fake_occupancy_grid.info.origin = msg.pose.pose
        fake_occupancy_grid.data = [1] * 200 * 200
        self.occupancy_grid_publisher.publish(fake_occupancy_grid)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
