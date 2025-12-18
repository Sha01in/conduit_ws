import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from conduit_interfaces.action import ExecuteMission


class OrchestratorNode(Node):
    """
    Orchestrates robot missions using ROS 2 Action Clients.

    This node acts as a central controller (simulating FactoryOS) that sends
    asynchronous goals to the factory robot and monitors their progress via
    feedback. It demonstrates robust cancellation logic to simulate operator
    intervention.
    """

    def __init__(self):
        """Initialize the OrchestratorNode with an action client."""
        super().__init__('orchestrator')
        self._action_client = ActionClient(self, ExecuteMission, 'execute_mission')
        self._goal_handle = None
        self._cancel_timer = None

    def send_mission(self, mission_id, target_coordinates):
        """
        Send a mission goal to the robot asynchronously.

        This method constructs the goal message and sends it to the action server.
        It attaches callbacks for feedback and the final result, ensuring the
        main thread remains unblocked for other orchestration tasks.

        Args:
            mission_id (str): Unique identifier for the mission.
            target_coordinates (list[float]): [x, y, theta] target.
        """
        goal_msg = ExecuteMission.Goal()
        goal_msg.mission_id = mission_id
        # In a real scenario, this would be dynamic or passed in.
        goal_msg.robot_id = 'default_robot'
        goal_msg.target_coordinates = target_coordinates

        self.get_logger().info('Waiting for action server to be available...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending mission {mission_id} to {target_coordinates}...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        Process partial feedback from the action server.

        Args:
            feedback_msg: Container with the feedback content.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: Status: {feedback.current_state}, '
            f'Progress: {feedback.progress_percentage}%')

    def goal_response_callback(self, future):
        """
        Handle the server's response to the goal request.

        If accepted, it sets up a timer to simulate a "Stop" command after
        3 seconds, demonstrating the system's ability to handle user
        cancellation mid-flight.

        Args:
            future: Future containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')

        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        # Timer to cancel the goal after 3 seconds
        self._cancel_timer = self.create_timer(3.0, self.timer_callback)

    def timer_callback(self):
        """Trigger cancellation of the current goal."""
        self.get_logger().info('Canceling mission after 3 seconds...')
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()
        if self._cancel_timer:
            self._cancel_timer.cancel()

    def get_result_callback(self, future):
        """
        Handle the final result of the action.

        Args:
            future: Future containing the wrapped result.
        """
        result = future.result().result
        self.get_logger().info(f'Result: Success={result.success}, '
                               f'Status={result.final_status}')


def main(args=None):
    rclpy.init(args=args)

    orchestrator = OrchestratorNode()

    # Example usage: sending a mission
    orchestrator.send_mission("MISSION_001", [10.0, 20.0, 0.0])

    rclpy.spin(orchestrator)

    orchestrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
