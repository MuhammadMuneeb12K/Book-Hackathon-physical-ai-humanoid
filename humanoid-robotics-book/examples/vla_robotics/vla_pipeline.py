import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from vision_msgs.msg import Detection2D, Detection2DArray
import time

# Conceptual ROS 2 Action messages for robot control
# from action_msgs.msg import GoalStatus
# from robot_action_interfaces.action import PickObject # Conceptual Action type

class VLAPipeline(Node):
    """
    A conceptual ROS 2 node that orchestrates the VLA pipeline:
    1. Subscribes to voice commands (text).
    2. Uses an LLM (mocked) to generate a plan.
    3. Uses visual detections (mocked) for object grounding.
    4. Executes the plan by sending conceptual robot actions.
    """
    def __init__(self):
        super().__init__('vla_pipeline')
        self.voice_command_subscription = self.create_subscription(
            String,
            'voice_command_text',
            self.voice_command_callback,
            10)
        self.object_detection_subscription = self.create_subscription(
            Detection2DArray,
            'object_detections',
            self.object_detection_callback,
            10)
        
        self.robot_action_publisher = self.create_publisher(KeyValue, 'robot_action_command', 10)
        
        self.current_plan = []
        self.detected_objects = {} # Stores detected objects for grounding
        
        self.get_logger().info('VLA Pipeline node started.')

    def voice_command_callback(self, msg: String):
        """
        Callback for incoming voice command text. Triggers planning.
        """
        command_text = msg.data
        self.get_logger().info(f'VLA: Received voice command: "{command_text}"')
        
        # Step 1: LLM-based planning (mocked)
        self.current_plan = self._mock_llm_plan_generation(command_text)
        self.get_logger().info(f'VLA: Generated plan: {self.current_plan}')
        
        if self.current_plan:
            self._execute_plan()

    def object_detection_callback(self, msg: Detection2DArray):
        """
        Callback for incoming object detections. Updates internal state.
        """
        self.detected_objects = {}
        for detection in msg.detections:
            class_name = "unknown" # Placeholder, assuming Detection2D doesn't have a direct class name
            # In a real setup, parse detection.results[0].id for class name
            if detection.results:
                 class_name = detection.results[0].id # Example for if result type had 'id'
            
            # Use bbox center_x and center_y for unique ID
            obj_key = f"{class_name}_{int(detection.bbox.center.x)}_{int(detection.bbox.center.y)}"
            self.detected_objects[obj_key] = detection
        self.get_logger().info(f'VLA: Updated detected objects: {len(self.detected_objects)} objects')

    def _mock_llm_plan_generation(self, command_text: str) -> list:
        """
        Mocks LLM planning. Ideally calls an LLM service.
        """
        if "pick up the blue block" in command_text.lower():
            return [
                {"action": "move_to", "target": "table"},
                {"action": "detect_object", "target": "blue block"},
                {"action": "grasp_object", "target": "blue block"},
                {"action": "move_to", "target": "red basket"},
                {"action": "release_object", "target": "blue block"}
            ]
        elif "move to the charging station" in command_text.lower():
            return [{"action": "move_to", "target": "charging station"}]
        return []

    def _execute_plan(self):
        """
        Executes the current plan step by step, incorporating visual grounding.
        """
        self.get_logger().info('VLA: Starting plan execution...')
        for step in self.current_plan:
            action = step["action"]
            target = step["target"]
            self.get_logger().info(f'VLA: Executing action: {action}, target: {target}')

            if action == "detect_object":
                # Simulate visual grounding: find target in detected_objects
                found = False
                for obj_key, det in self.detected_objects.items():
                    if target in obj_key.lower(): # Simple substring match for mock
                        self.get_logger().info(f'VLA: Grounded "{target}" to {obj_key} at ({det.bbox.center.x}, {det.bbox.center.y})')
                        found = True
                        break
                if not found:
                    self.get_logger().warn(f'VLA: Could not ground "{target}" visually. Plan may fail.')
                    # In real robot, would initiate replanning or error state
            
            # Publish conceptual robot action to be handled by a robot controller
            action_msg = KeyValue()
            action_msg.key = action
            action_msg.value = str(target) if target is not None else ""
            self.robot_action_publisher.publish(action_msg)
            
            # Simulate action duration
            time.sleep(2) # Blocking for simplicity, in real ROS 2 this would be async

        self.get_logger().info('VLA: Plan execution finished.')
        self.current_plan = [] # Reset plan


def main(args=None):
    rclpy.init(args=args)
    try:
        vla_pipeline = VLAPipeline()
        rclpy.spin(vla_pipeline)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        vla_pipeline.get_logger().error(f"An error occurred: {e}")
    finally:
        if 'vla_pipeline' in locals() and rclpy.ok():
            vla_pipeline.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
