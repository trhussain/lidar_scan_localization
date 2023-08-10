import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs as pc2
import math
import struct 
import laser_geometry.laser_geometry as lg
import os 

class LidarPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.sub = self.create_subscription(PointCloud2, 'lidar_0/m1600/pcl2', self.pointcloud_callback, 10)
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)
        
        self.laser_projector = lg.LaserProjection()
        #self.timer = self.create_timer(1.0, self.publish_lidar_data)
        
        
    def pc_callback_test2(self,msg): # Different attempt at conversion -> big error crashes 
        lc_msg = self.laser_projector.projectLaser(msg)
        self.publisher.publish(lc_msg)
        self.get_logger().info('laserscan published')

        
    def frange(self, start, end, step):
        while start <= end:
            yield start
            start += step
     
    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to a list of (x, y, z) points
        points = list(self.unpack_points(msg, skip_nans=True, field_names=("x", "y", "z")))

        # Set the LaserScan message properties
        laserscan_msg = LaserScan()
        laserscan_msg.header = msg.header
        laserscan_msg.angle_min = 0  # Minimum angle in radians
        laserscan_msg.angle_max = math.pi * 2.0   # Maximum angle in radians
        laserscan_msg.angle_increment = math.pi / len(points)  # Angular increment between measurements
        laserscan_msg.range_min = 0.0             # Minimum range value
        laserscan_msg.range_max = 100.0           # Maximum range value (adjust as needed)
        laserscan_msg.ranges = []                 # LaserScan data

        # Project the 3D points onto the XY plane and find the nearest points in each angular sector
        for angle in self.frange(laserscan_msg.angle_min, laserscan_msg.angle_max, laserscan_msg.angle_increment):
            min_dist = float('inf')
            for point in points:
                x, y, z = point
                dist = math.sqrt(x**2 + y**2)
                angle_point = math.atan2(y, x)
                if math.isclose(angle_point, angle, abs_tol=laserscan_msg.angle_increment / 2.0) and dist < min_dist:
                    min_dist = dist
            laserscan_msg.ranges.append(min_dist)

        # Publish the LaserScan message
        self.publisher.publish(laserscan_msg)

    def unpack_points(self, pc_msg, skip_nans=True, field_names=("x", "y", "z")):
        field_names = set(field_names)
        point_fields = pc_msg.fields
        point_step = pc_msg.point_step
        data_buffer = pc_msg.data

        for offset in range(0, len(data_buffer), point_step):
            point = []
            for field in point_fields:
                if field.name in field_names:
                    point.append(self.read_point_field(field, data_buffer, offset))

            if skip_nans and any(math.isnan(coord) for coord in point):
                continue

            yield tuple(point)

    @staticmethod
    def read_point_field(field, data_buffer, offset):
        """Reads a single field from a binary blob of a point.

        Args:
            field (PointField): The PointField describing the field.
            data_buffer (bytes): The binary data of the point cloud.
            offset (int): The offset at which the field starts.

        Returns:
            Union[float, int]: The value of the field.
        """

        field_map = {'F': ('f', 4), 'I': ('I', 4), 'i': ('i', 4),
                    'B': ('B', 1), 'b': ('b', 1), 'd': ('d', 8), 'u': ('H', 2), 7: ('d', 8)}

        fmt, size = field_map[field.datatype]
        return struct.unpack_from(fmt, data_buffer, offset + field.offset)[0] if fmt != 'x' else None


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
