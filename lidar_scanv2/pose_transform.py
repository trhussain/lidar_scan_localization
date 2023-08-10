import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry



class PoseTransform(Node): 
    def __init__(self):
        super().__init__('PoseTransform')
        self.sub = self.create_subscription(Odometry, '/zed2i/zed_node/odom', self.odom2, 10)
        self.publisher = self.create_publisher(Odometry, 'odom2', 10)
    
    def odom2(self,msg) -> None: # Different attempt at conversion -> big error crashes
        msg2 = self.odom_edit(msg)
        self.publisher.publish(msg2)
        #self.get_logger().info('odom published')
      
        
        
        
        
        
    def odom_edit(self,msg):
        msg.pose.pose.position.x = msg.pose.pose.position.x - 250
        msg.pose.pose.position.y = msg.pose.pose.position.y - 50
        msg.pose.pose.position.z = msg.pose.pose.position.z - 50
        return msg 









def main(args=None):
    rclpy.init(args=args)
    node = PoseTransform()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
