from typing import TypedDict, List, Dict, Optional
from enum import Enum


class TaskStatus(Enum):
    PENDING = "pending"
    DETECTING = "detecting"
    CLASSIFYING = "classifying"
    PLANNING = "planning"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"


class BlockInfo(TypedDict):
    id: int
    tool_pos: List[float]
    color: str
    base_pos: Optional[List[float]]
    grasped: bool


class GraspCommand(TypedDict):
    color: str
    num: int
    other_num: Optional[int]


class GraspState(TypedDict):
    status: TaskStatus
    command: Optional[GraspCommand]
    raw_blocks: List[BlockInfo]
    target_color_blocks: List[BlockInfo]
    other_color_blocks: List[BlockInfo]
    grasp_queue: List[BlockInfo]
    current_block: Optional[BlockInfo]
    grasped_count: int
    total_to_grasp: int
    current_pose: Optional[List[float]]
    detected_blocks: List[BlockInfo]
    robot_interface: Optional[object]
    error_message: Optional[str]
    retry_count: int
    max_retries: int