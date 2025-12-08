import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self._action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = FollowWaypoints.Goal()
        
        # --- Define your waypoints here ---
        # Each waypoint is a PoseStamped message.
        # You can add as many waypoints as you need.
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = 'map'
        waypoint1.header.stamp = self.get_clock().now().to_msg()
        waypoint1.pose.position.x = 1.0
        waypoint1.pose.position.y = 1.0
        waypoint1.pose.orientation.w = 1.0

        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = 'map'
        waypoint2.header.stamp = self.get_clock().now().to_msg()
        waypoint2.pose.position.x = 2.0
        waypoint2.pose.position.y = 1.0
        waypoint2.pose.orientation.w = 1.0

        goal_msg.poses = [waypoint1, waypoint2]
        # ------------------------------------

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.missed_waypoints}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.current_waypoint}')

def main(args=None):
    rclpy.init(args=args)
    action_client = WaypointFollower()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()