import rclpy
from std_msgs.msg import Empty
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped

class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_pose_action_client')
        
        
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        # 订阅目标位姿主题
        self.subscriber_ = self.create_subscription(
            PoseStamped, 
            '/goal_pose', 
            self.goal_pose_callback, 
            1
        )
        self.mark_pub = self.create_publisher(String, '/move_base/status', 1)
        

    def goal_pose_callback(self, msg):
        goal_position = msg.pose.position
        self.send_goal(goal_position.x, goal_position.y, 0.0)

    def send_goal(self, x, y, theta):
        #self.get_logger().info('Sending goal to action server')
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.position.z = 0.0  # Z 坐标通常为 0
        goal_pose.pose.pose.orientation.w = 1.0  # 没有旋转的情况下设置为单位四元数

        #self.get_logger().info('Waiting for action server')
        self._action_client.wait_for_server()
        #self.get_logger().info('Action server detected')

        self._send_goal_future = self._action_client.send_goal_async(goal_pose)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            #self.get_logger().info('Goal rejected :(')
            return
        #self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(str(result))
        if isinstance(result, Empty):
            msg = String()
            msg.data = 'success'
            self.mark_pub.publish(msg)
            self.get_logger().info(str(msg))
            
        
        #self.get_logger().info('Result: {0}'.format(result))
        #rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavToPoseActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
