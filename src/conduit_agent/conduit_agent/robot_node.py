import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from conduit_interfaces.action import ExecuteMission

class RobotNode(Node):

    def __init__(self):
        super().__init__('factory_robot')
        self._action_server = ActionServer(
            self,
            ExecuteMission,
            'execute_mission',
            self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info('Factory Robot Node Started')

    def goal_callback(self, goal_request):
        """Accept or reject a goal request."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')
        
        # Log mission_id
        mission_id = goal_handle.request.mission_id
        self.get_logger().info(f'Mission ID: {mission_id}')

        feedback_msg = ExecuteMission.Feedback()
        
        # Simulate 10-second task
        for i in range(1, 11):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = ExecuteMission.Result()
                result.success = False
                result.final_status = "CANCELED"
                return result

            # Publish feedback
            feedback_msg.progress_percentage = float(i * 10)
            feedback_msg.current_state = "WORKING"
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.progress_percentage}%')
            
            time.sleep(1.0)

        # Success
        goal_handle.succeed()
        
        result = ExecuteMission.Result()
        result.success = True
        result.final_status = "COMPLETED"
        return result

def main(args=None):
    rclpy.init(args=args)
    
    action_server = RobotNode()
    
    # Use MultiThreadedExecutor so callbacks (like cancel) can run in parallel 
    # with the long-running execute_callback
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)
    
    action_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
