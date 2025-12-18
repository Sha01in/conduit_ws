import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from conduit_interfaces.action import ExecuteMission


class RobotNode(Node):
    """
    Simulates a factory robot using a ROS 2 Action Server.

    This node implements the 'ExecuteMission' action server, simulating a
    long-running task with feedback and cancellation support. It uses a
    MultiThreadedExecutor to handle cancellation requests asynchronously.
    """

    def __init__(self):
        """Initialize the RobotNode with an action server."""
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
        """
        Accept or reject a goal request.

        Args:
            goal_request: The goal request message.

        Returns:
            GoalResponse.ACCEPT to indicate the goal is accepted.
        """
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept or reject a cancel request.

        Args:
            goal_handle: The handle to the goal being cancelled.

        Returns:
            CancelResponse.ACCEPT to indicate cancellation is allowed.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute the mission goal.

        This method simulates a long-running task (10 seconds) by iterating
        through a loop and sleeping. It publishes feedback every second.

        Async Logic:
            The execution loop explicitly checks `goal_handle.is_cancel_requested`
            to support non-blocking cancellation. This ensures that an "E-Stop"
            signal can immediately arrest the process, returning a proper
            CANCELED state instead of ABORTED.

        Args:
            goal_handle: The goal handle for the current execution.

        Returns:
            ExecuteMission.Result: The result of the mission (Success or Canceled).
        """
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
    # with the long-running execute_callback.
    # This architecture prevents the blocking execute_callback from starving the
    # executor, ensuring that 'cancel_callback' can be processed immediately.
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
