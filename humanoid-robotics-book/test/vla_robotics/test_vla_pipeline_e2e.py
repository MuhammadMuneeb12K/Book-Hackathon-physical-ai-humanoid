import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import KeyValue
from vision_msgs.msg import Detection2D, Detection2DArray, BoundingBox2D
import time
import threading

# Conceptual imports to simulate other nodes
# from examples.vla_robotics.whisper_translator import WhisperTranslator
# from examples.vla_robotics.llm_planner import LLMPlanner
# from examples.vla_robotics.object_detector import ObjectDetector
# from examples.vla_robotics.vla_pipeline import VLAPipeline


class TestVLAPipelineE2E(unittest.TestCase):

    publisher_node = None
    vla_pipeline_node = None
    executor_thread = None
    
    @classmethod
    def setUpClass(cls):
        """
        Initialize ROS 2 and set up conceptual VLA pipeline nodes.
        """
        print("\nSetting up ROS 2 for VLA pipeline E2E tests...")
        rclpy.init(args=None)
        
        # Publisher node to simulate external inputs (voice commands, detections)
        cls.publisher_node = rclpy.create_node('e2e_test_publisher_node')
        cls.voice_pub = cls.publisher_node.create_publisher(String, 'voice_command_text', 10)
        cls.detection_pub = cls.publisher_node.create_publisher(Detection2DArray, 'object_detections', 10)

        # Subscriber for robot actions
        cls.received_robot_actions = []
        cls.action_sub = cls.publisher_node.create_subscription(
            KeyValue,
            'robot_action_command',
            lambda msg: cls.received_robot_actions.append(msg),
            10
        )

        # Conceptual VLA Pipeline Node
        # In a real test, you would instantiate the actual VLA Pipeline node
        # For simplicity and to avoid circular imports, we'll simulate its behavior
        # or assume it's running externally. 
        # Here, we'll run a minimal VLA Pipeline mock for its output.
        from examples.vla_robotics.vla_pipeline import VLAPipeline
        cls.vla_pipeline_node = VLAPipeline() # The actual VLA pipeline logic being tested
        
        # Use a MultiThreadedExecutor to spin nodes in the background
        cls.executor = rclpy.executors.MultiThreadedExecutor()
        cls.executor.add_node(cls.publisher_node)
        cls.executor.add_node(cls.vla_pipeline_node) # Add the node under test
        
        cls.executor_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()

        print("ROS 2 test nodes and executor started.")
        time.sleep(2) # Give nodes time to set up subscriptions/publishers

    @classmethod
    def tearDownClass(cls):
        """
        Shutdown ROS 2 after all tests.
        """
        print("\nTearing down ROS 2 resources...")
        cls.executor.shutdown()
        cls.executor_thread.join()
        
        if cls.publisher_node:
            cls.publisher_node.destroy_node()
        if cls.vla_pipeline_node:
            cls.vla_pipeline_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 test nodes destroyed and ROS 2 shut down.")

    def setUp(self):
        # Clear received actions before each test
        self.received_robot_actions.clear()
        time.sleep(0.5) # Small delay to ensure clear state

    def test_e2e_pick_and_place_scenario(self):
        """
        Test a full pick and place scenario: Voice -> LLM -> Vision -> Action.
        """
        print("--- Running E2E Pick and Place Scenario ---")
        
        # 1. Simulate Voice Command (Text from Whisper)
        voice_cmd = String()
        voice_cmd.data = "Robot, pick up the blue block from the table and put it in the basket"
        self.voice_pub.publish(voice_cmd)
        print(f"Published voice command: '{voice_cmd.data}'")
        time.sleep(1) # Give LLM Planner time to process

        # 2. Simulate Object Detections (from YOLO/SAM)
        det_array_msg = Detection2DArray()
        det_array_msg.header.stamp = self.publisher_node.get_clock().now().to_msg()
        
        blue_block_det = Detection2D()
        blue_block_det.bbox.center.x = 320.0
        blue_block_det.bbox.center.y = 240.0
        blue_block_det.bbox.size_x = 50.0
        blue_block_det.bbox.size_y = 50.0
        blue_block_det.results.append(String(data="blue block")) # Conceptual class name
        det_array_msg.detections.append(blue_block_det)

        red_basket_det = Detection2D()
        red_basket_det.bbox.center.x = 100.0
        red_basket_det.bbox.center.y = 400.0
        red_basket_det.bbox.size_x = 80.0
        red_basket_det.bbox.size_y = 80.0
        red_basket_det.results.append(String(data="red basket")) # Conceptual class name
        det_array_msg.detections.append(red_basket_det)

        self.detection_pub.publish(det_array_msg)
        print(f"Published {len(det_array_msg.detections)} object detections.")
        time.sleep(5) # Give VLA pipeline time to process plan and detections

        # 3. Verify Robot Actions
        expected_actions = [
            ("move_to", "table"),
            ("detect_object", "blue block"),
            ("grasp_object", "blue block"),
            ("move_to", "red basket"),
            ("release_object", "blue block")
        ]

        self.assertGreaterEqual(len(self.received_robot_actions), len(expected_actions), 
                                "Not enough robot actions received.")
        
        for i, (expected_key, expected_value) in enumerate(expected_actions):
            if i < len(self.received_robot_actions):
                received_action = self.received_robot_actions[i]
                self.assertEqual(received_action.key, expected_key, 
                                f"Action {i} key mismatch. Expected '{expected_key}', got '{received_action.key}'.")
                self.assertEqual(received_action.value, expected_value, 
                                f"Action {i} value mismatch. Expected '{expected_value}', got '{received_action.value}'.")
            else:
                self.fail(f"Expected action {i} ({expected_key}: {expected_value}) but no more actions received.")
        
        print("E2E Pick and Place Scenario PASSED.")

if __name__ == '__main__':
    # To run this conceptual test:
    # 1. Ensure a ROS 2 environment is sourced.
    # 2. Run: python -m unittest test.vla_robotics.test_vla_pipeline_e2e
    unittest.main()
