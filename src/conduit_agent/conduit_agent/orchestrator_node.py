import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from conduit_interfaces.action import ExecuteMission

class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator')
        self._action_client = ActionClient(self, ExecuteMission, 'execute_mission')

    def send_mission(self, mission_id, target_coordinates):
        goal_msg = ExecuteMission.Goal()
        goal_msg.mission_id = mission_id
        # goal_msg.robot_id = robot_id # Assuming robot_id might be needed but prompt only asked for id and coords in method signature. 
        # Using placeholder or omitting optional fields if they were in the action.
        # Checking action definition from memory: 
        # string mission_id
        # string robot_id
        # float32[] target_coordinates
        
        # I'll add a default robot_id since the method signature requested only id and coords.
        goal_msg.robot_id = 'default_robot' 
        goal_msg.target_coordinates = target_coordinates

        self.get_logger().info(f'Waiting for action server to be available...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending mission {mission_id} to {target_coordinates}...')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: Status: {feedback.current_state}, Progress: {feedback.progress_percentage}%')

    def goal_response_callback(self, future):
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
        self.get_logger().info('Canceling mission after 3 seconds...')
        self._goal_handle.cancel_goal_async()
        self._cancel_timer.cancel()

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Success={result.success}, Status={result.final_status}')
        
        # Shutdown after result (optional, but good for testing)
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    orchestrator = OrchestratorNode()

    # Example usage: sending a mission
    # In a real scenario this might be triggered by an external event or service call.
    orchestrator.send_mission("MISSION_001", [10.0, 20.0, 0.0])

    rclpy.spin(orchestrator)

    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
