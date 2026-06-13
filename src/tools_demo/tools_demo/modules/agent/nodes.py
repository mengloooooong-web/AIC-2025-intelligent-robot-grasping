import logging
from typing import Dict, Literal, Any
from .state import GraspState, TaskStatus, BlockInfo
from ..tools import tool_to_base
import numpy as np

logger = logging.getLogger(__name__)


def detect_blocks_node(state: GraspState) -> GraspState:
    state["status"] = TaskStatus.DETECTING
    logger.info("Agent: Detecting blocks...")

    detected_blocks = state.get("detected_blocks", [])
    
    if not detected_blocks:
        state["status"] = TaskStatus.FAILED
        state["error_message"] = "No blocks detected from perception system"
        return state

    state["raw_blocks"] = detected_blocks
    state["status"] = TaskStatus.CLASSIFYING
    logger.info(f"Agent: Detected {len(detected_blocks)} blocks")
    return state


def classify_blocks_node(state: GraspState) -> GraspState:
    state["status"] = TaskStatus.CLASSIFYING
    logger.info("Agent: Classifying blocks by color...")

    command = state.get("command")
    if not command:
        state["status"] = TaskStatus.FAILED
        state["error_message"] = "No command found for classification"
        return state

    target_color = command.get("color", "")
    raw_blocks = state.get("raw_blocks", [])

    target_blocks = []
    other_blocks = []

    for block in raw_blocks:
        block: BlockInfo
        block_color = block.get("color", "").lower()
        target_color_lower = target_color.lower()
        
        if block_color == target_color_lower:
            target_blocks.append(block)
        else:
            other_blocks.append(block)

    state["target_color_blocks"] = target_blocks
    state["other_color_blocks"] = other_blocks
    state["status"] = TaskStatus.PLANNING

    logger.info(f"Agent: Classified {len(target_blocks)} target blocks ({target_color}), "
                f"{len(other_blocks)} other blocks")
    return state


def plan_grasp_node(state: GraspState) -> GraspState:
    state["status"] = TaskStatus.PLANNING
    logger.info("Agent: Planning grasp sequence...")

    command = state.get("command")
    if not command:
        state["status"] = TaskStatus.FAILED
        state["error_message"] = "No command found for planning"
        return state

    target_num = command.get("num", 0)
    target_color_blocks = state.get("target_color_blocks", [])
    other_blocks = state.get("other_color_blocks", [])

    grasp_queue = []

    sorted_target = _sort_blocks_by_priority(target_color_blocks)
    grasp_queue.extend(sorted_target[:target_num])

    other_num = command.get("other_num", 0) if "other_num" in command else 0
    sorted_other = _sort_blocks_by_priority(other_blocks)
    grasp_queue.extend(sorted_other[:other_num])

    current_pose = state.get("current_pose")
    if current_pose:
        for block in grasp_queue:
            block["grasped"] = False
            block["base_pos"] = tool_to_base(block.get("tool_pos", []), current_pose)
    else:
        for block in grasp_queue:
            block["grasped"] = False
            block["base_pos"] = None

    state["grasp_queue"] = grasp_queue
    state["total_to_grasp"] = len(grasp_queue)
    state["grasped_count"] = 0
    state["status"] = TaskStatus.EXECUTING

    logger.info(f"Agent: Planned grasp sequence for {len(grasp_queue)} blocks")
    return state


def _sort_blocks_by_priority(blocks: list) -> list:
    def calculate_priority(block: BlockInfo) -> float:
        pos = block.get("tool_pos", [0, 0, 0])
        x, y, z = pos[0], pos[1], pos[2]
        distance = np.sqrt(x**2 + y**2 + z**2)
        reachability_score = 1.0 / (distance + 0.1)
        z_score = z / 1000.0 if z > 0 else 0
        return reachability_score + z_score

    return sorted(blocks, key=calculate_priority, reverse=True)


def execute_grasp_node(state: GraspState) -> GraspState:
    state["status"] = TaskStatus.EXECUTING
    logger.info("Agent: Executing grasp...")

    grasp_queue = state.get("grasp_queue", [])
    if not grasp_queue:
        state["status"] = TaskStatus.COMPLETED
        return state

    current_block = None
    for block in grasp_queue:
        if not block.get("grasped", False):
            current_block = block
            break

    if current_block is None:
        state["status"] = TaskStatus.COMPLETED
        return state

    robot_interface = state.get("robot_interface")
    
    try:
        if robot_interface:
            base_pos = current_block.get("base_pos")
            if base_pos:
                success = _execute_grasp_with_robot(robot_interface, base_pos, current_block.get("color", ""))
            else:
                success = False
                state["error_message"] = "Block position not available"
        else:
            success = _execute_grasp_simulation(current_block)
        
        if success:
            current_block["grasped"] = True
            state["grasped_count"] += 1
            state["retry_count"] = 0
            logger.info(f"Agent: Successfully grasped block {current_block.get('id', 'unknown')}")
        else:
            state["retry_count"] = state.get("retry_count", 0) + 1
            logger.warning(f"Agent: Failed to grasp block {current_block.get('id', 'unknown')}, retry {state['retry_count']}")

            if state["retry_count"] >= state.get("max_retries", 3):
                state["error_message"] = f"Max retries exceeded for block {current_block.get('id', 'unknown')}"
                state["status"] = TaskStatus.FAILED
                return state

    except Exception as e:
        state["retry_count"] = state.get("retry_count", 0) + 1
        logger.error(f"Agent: Exception during grasp: {str(e)}")
        if state["retry_count"] >= state.get("max_retries", 3):
            state["error_message"] = f"Exception: {str(e)}"
            state["status"] = TaskStatus.FAILED
            return state

    state["current_block"] = current_block
    return state


def _execute_grasp_with_robot(robot, base_pos: list, color: str) -> bool:
    try:
        x, y, z = base_pos
        offset_x = 8.5
        offset_y = -53
        
        ret = robot.go_point([x * 1000 + offset_x, y * 1000 + offset_y, 155.0])
        if ret[0] != 0:
            return False
        
        robot.do_pick_on(-8)
        
        back_pose = [-100, 340, 200, 180.0, 0.0, 0.0]
        ret = robot.go_pose(back_pose)
        if ret[0] != 0:
            return False
        
        robot.pick_off()
        return True
    except Exception as e:
        logger.error(f"Robot grasp error: {e}")
        return False


def _execute_grasp_simulation(block: BlockInfo) -> bool:
    logger.info(f"Simulating grasp for block {block.get('id')} at {block.get('tool_pos')}")
    return True


def check_complete_node(state: GraspState) -> Literal["execute_grasp", "completed"]:
    grasp_queue = state.get("grasp_queue", [])
    grasped_count = state.get("grasped_count", 0)
    total_to_grasp = state.get("total_to_grasp", 0)

    remaining = [b for b in grasp_queue if not b.get("grasped", False)]

    if grasped_count >= total_to_grasp or not remaining:
        state["status"] = TaskStatus.COMPLETED
        logger.info("Agent: All grasp tasks completed!")
        return "completed"

    logger.info(f"Agent: {len(remaining)} blocks remaining, continuing grasp...")
    return "execute_grasp"


def should_retry(state: GraspState) -> Literal["execute_grasp", "failed"]:
    retry_count = state.get("retry_count", 0)
    max_retries = state.get("max_retries", 3)

    if retry_count >= max_retries:
        state["status"] = TaskStatus.FAILED
        logger.error(f"Agent: Max retries ({max_retries}) exceeded")
        return "failed"

    return "execute_grasp"


def initialize_state(command: Dict, detected_blocks: list = None, current_pose: list = None, robot_interface=None) -> GraspState:
    return GraspState(
        status=TaskStatus.PENDING,
        command=command,
        raw_blocks=[],
        target_color_blocks=[],
        other_color_blocks=[],
        grasp_queue=[],
        current_block=None,
        grasped_count=0,
        total_to_grasp=0,
        current_pose=current_pose,
        detected_blocks=detected_blocks or [],
        robot_interface=robot_interface,
        error_message=None,
        retry_count=0,
        max_retries=3,
    )