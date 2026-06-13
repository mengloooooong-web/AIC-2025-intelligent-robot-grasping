from typing import Literal, Dict
from langgraph.graph import StateGraph, END
from langgraph.graph.state import CompiledStateGraph
from .state import GraspState, TaskStatus
from .nodes import (
    detect_blocks_node,
    classify_blocks_node,
    plan_grasp_node,
    execute_grasp_node,
    initialize_state,
)


def create_grasp_graph() -> CompiledStateGraph:
    workflow = StateGraph(GraspState)

    workflow.add_node("detect_blocks", detect_blocks_node)
    workflow.add_node("classify_blocks", classify_blocks_node)
    workflow.add_node("plan_grasp", plan_grasp_node)
    workflow.add_node("execute_grasp", execute_grasp_node)

    workflow.set_entry_point("detect_blocks")

    workflow.add_edge("detect_blocks", "classify_blocks")
    workflow.add_edge("classify_blocks", "plan_grasp")
    workflow.add_edge("plan_grasp", "execute_grasp")

    def check_complete(state: GraspState) -> Literal["execute_grasp", END]:
        grasp_queue = state.get("grasp_queue", [])
        grasped_count = state.get("grasped_count", 0)
        total_to_grasp = state.get("total_to_grasp", 0)

        remaining = [b for b in grasp_queue if not b.get("grasped", False)]

        if grasped_count >= total_to_grasp or not remaining:
            state["status"] = TaskStatus.COMPLETED
            return END
        return "execute_grasp"

    def should_retry(state: GraspState) -> Literal["execute_grasp", END]:
        if state.get("status") == TaskStatus.FAILED:
            return END
        retry_count = state.get("retry_count", 0)
        max_retries = state.get("max_retries", 3)

        if retry_count >= max_retries:
            state["status"] = TaskStatus.FAILED
            return END
        return "execute_grasp"

    workflow.add_conditional_edges(
        "execute_grasp",
        check_complete,
        {
            "execute_grasp": "execute_grasp",
            END: END,
        },
    )

    return workflow.compile()


class GraspAgent:
    def __init__(self):
        self.graph = create_grasp_graph()

    def run(self, command: dict) -> GraspState:
        initial_state = initialize_state(command)
        result = self.graph.invoke(initial_state)
        return result

    def run_with_state(self, initial_state: GraspState) -> GraspState:
        result = self.graph.invoke(initial_state)
        return result

    async def run_async(self, command: dict) -> GraspState:
        initial_state = initialize_state(command)
        result = await self.graph.ainvoke(initial_state)
        return result

    async def run_async_with_state(self, initial_state: GraspState) -> GraspState:
        result = await self.graph.ainvoke(initial_state)
        return result

    def get_graph_state(self) -> dict:
        return {
            "nodes": [
                "detect_blocks",
                "classify_blocks",
                "plan_grasp",
                "execute_grasp",
            ],
            "edges": [
                ("detect_blocks", "classify_blocks"),
                ("classify_blocks", "plan_grasp"),
                ("plan_grasp", "execute_grasp"),
                ("execute_grasp", "execute_grasp"),
                ("execute_grasp", END),
            ],
        }