import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf_transformations

class SequentialActionExecutor(Node):
    def __init__(self):
        super().__init__('sequential_action_executor')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoints = self.get_waypoints()
        self.current_waypoint_index = 0

    def get_waypoints(self):
        # 定义航点列表
        waypoints = [
            {'x': 0.0, 'y': 10.0, 'z': -5.0, 'yaw': 0.0},
            {'x': 7.071, 'y': 7.071, 'z': -5.0, 'yaw': 0.0},
            {'x': 10.0, 'y': 0.0, 'z': -5.0, 'yaw': 0.0},
            {'x': 7.071, 'y': -7.071, 'z': -5.0, 'yaw': 0.0},
            {'x': 0.0, 'y': -10.0, 'z': -5.0, 'yaw': 0.0},
            {'x': -7.071, 'y': -7.071, 'z': -5.0, 'yaw': 0.0},
            {'x': -10.0, 'y': 0.0, 'z': -5.0, 'yaw': 0.0},
            {'x': -7.071, 'y': 7.071, 'z': -5.0, 'yaw': 0.0}
        ]
        return waypoints

    def send_goal(self, waypoint):
        # 创建目标Pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoint['x']
        goal_pose.pose.position.y = waypoint['y']
        goal_pose.pose.position.z = waypoint['z']

        # 设置姿态（使用四元数表示）
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, waypoint['yaw'])
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        # 发送目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client.wait_for_server()
        self.get_logger().info(f"Sending goal: {waypoint}")
        self.action_client.send_goal(goal_msg, feedback_callback=self.feedback_callback)
        future = self.action_client.get_result_async()
        return future

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback}")

    def execute_waypoints(self):
        # 按顺序执行航点
        for waypoint in self.waypoints:
            future = self.send_goal(waypoint)
            rclpy.spin_until_future_complete(self, future)
            result = future.result().result
            self.get_logger().info(f"Result: {result}")
            if not result:
                self.get_logger().error("Goal failed")
                break
        self.get_logger().info("All waypoints executed")

def main(args=None):
    rclpy.init(args=args)
    node = SequentialActionExecutor()
    node.execute_waypoints()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()