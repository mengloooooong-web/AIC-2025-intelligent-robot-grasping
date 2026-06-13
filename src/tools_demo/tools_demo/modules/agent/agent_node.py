import rclpy
from rclpy.node import Node
from custom_msgs.srv import StrMsg
from std_srvs.srv import Trigger
from ai_msgs.msg import PerceptionTargets, Target, Roi
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import json
import logging
from time import sleep
from threading import Thread

from .state import GraspState
from .graph import GraspAgent
from ..tools import tool_to_base, get_color, pixel_to_camera, camera_to_tool, create_handeye_matrix
from ..jaka import Jaka
from .nodes import initialize_state
from cv_bridge import CvBridge

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LangGraphAgentNode(Node):
    def __init__(self):
        super().__init__("langgraph_agent_node")
        self.get_logger().info("LangGraph Agent Node initialized")

        self.agent = GraspAgent()
        self.robot = Jaka()

        if not self.robot.login_state:
            self.get_logger().error("Robot login failed")
        else:
            self.get_logger().info("Robot login successful")

        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None

        self.detected_blocks = []
        self.current_pose = None

        self.detection_sub = self.create_subscription(
            PerceptionTargets,
            "/hobot_dnn_detection",
            self.detection_callback,
            10,
        )

        self.depth_sub = self.create_subscription(
            Image,
            "/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10,
        )

        self.color_sub = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.color_callback,
            10,
        )

        self.command_sub = self.create_subscription(
            String,
            "/grasp_command",
            self.command_callback,
            10,
        )

        self.command_srv = self.create_service(
            StrMsg,
            "/agent_send_command",
            self.command_service_callback,
        )

        self.detection_service = self.create_service(
            Trigger,
            "/restart_detection",
            self.restart_detection_callback,
        )

        self.get_logger().info("LangGraph Agent is ready")

    def detection_callback(self, msg: PerceptionTargets):
        self.detected_blocks = []

        for target in msg.targets:
            for roi in target.rois:
                if roi.type != "block" or roi.confidence < 0.55:
                    continue

                x_offset = roi.rect.x_offset
                y_offset = roi.rect.y_offset
                width = roi.rect.width
                height = roi.rect.height

                center_x = x_offset + width / 2
                center_y = y_offset + height / 2

                depth_value = self.get_depth(center_x, center_y)
                color = self.get_block_color(x_offset, y_offset, width, height)

                if depth_value is None:
                    depth_value = 0.0

                T_tool_cam = create_handeye_matrix()
                point_3d = pixel_to_camera(center_x, center_y, depth_value)
                point_3d_tool = camera_to_tool(point_3d[0], point_3d[1], point_3d[2], T_tool_cam)

                if self.is_new_block(point_3d_tool):
                    block = {
                        "id": len(self.detected_blocks),
                        "tool_pos": [point_3d_tool[0], point_3d_tool[1], point_3d_tool[2]],
                        "color": color,
                        "base_pos": None,
                        "grasped": False,
                    }
                    self.detected_blocks.append(block)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def color_callback(self, msg):
        try:
            self.color_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Color image conversion error: {e}")

    def command_callback(self, msg: String):
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f"Received command: {command}")
            self.execute_grasp_task(command)
        except Exception as e:
            self.get_logger().error(f"Command parse error: {e}")

    def command_service_callback(self, request: StrMsg.Request, response: StrMsg.Response):
        try:
            command = json.loads(request.data)
            self.get_logger().info(f"Received command via service: {command}")

            thread = Thread(target=self.execute_grasp_task, args=(command,))
            thread.start()

            response.success = True
            response.message = "Command received, processing started"
        except Exception as e:
            self.get_logger().error(f"Service callback error: {e}")
            response.success = False
            response.message = str(e)

        return response

    def restart_detection_callback(self, request: Trigger.Request, response: Trigger.Response):
        self.get_logger().info("Restarting detection...")

        blocks_data = []
        for block in self.detected_blocks:
            blocks_data.append({
                "id": block["id"],
                "tool_pos": block["tool_pos"],
                "color": block["color"],
            })

        response.success = True
        response.message = json.dumps(blocks_data)
        return response

    def execute_grasp_task(self, command: dict):
        self.get_logger().info("Starting LangGraph Agent task...")

        self.current_pose = self.robot.get_tools_pos()
        if self.current_pose == -1:
            self.get_logger().error("Failed to get robot pose")
            return

        initial_state = initialize_state(
            command=command,
            detected_blocks=self.detected_blocks,
            current_pose=self.current_pose,
            robot_interface=self.robot
        )

        result = self.agent.run_with_state(initial_state)

        self.process_agent_result(result)

    def process_agent_result(self, result: GraspState):
        status = result.get("status")
        self.get_logger().info(f"Agent task completed with status: {status}")

        if status and status.value == "completed":
            grasped_count = result.get("grasped_count", 0)
            total = result.get("total_to_grasp", 0)
            self.get_logger().info(f"Successfully grasped {grasped_count}/{total} blocks")
            self.robot.go_home()
        elif status and status.value == "failed":
            error = result.get("error_message", "Unknown error")
            self.get_logger().error(f"Agent task failed: {error}")

    def get_depth(self, x, y) -> float:
        if self.depth_image is None:
            return None

        height, width = self.depth_image.shape
        if 0 <= x < width and 0 <= y < height:
            return self.depth_image[int(y), int(x)] / 1000.0
        return None

    def get_block_color(self, x, y, w, h) -> str:
        if self.color_image is None:
            return "unknown"
        
        try:
            return get_color(self.color_image, [int(x), int(y), int(w), int(h)])
        except Exception as e:
            self.get_logger().error(f"Color detection error: {e}")
            return "unknown"

    def is_new_block(self, position, min_distance=0.05):
        for block in self.detected_blocks:
            existing_pos = block["tool_pos"]
            distance = ((position[0] - existing_pos[0])**2 +
                       (position[1] - existing_pos[1])**2 +
                       (position[2] - existing_pos[2])**2)**0.5
            if distance < min_distance:
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = LangGraphAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()