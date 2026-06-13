import sys
sys.path.insert(0, '.')

from tools_demo.modules.agent.graph import GraspAgent
from tools_demo.modules.agent.nodes import initialize_state

def test_agent():
    print("Testing LangGraph Agent for two-color block grasping...")
    
    agent = GraspAgent()
    print("✅ Agent created successfully")
    
    test_blocks = [
        {"id": 0, "tool_pos": [0.1, 0.2, 0.05], "color": "yellow", "base_pos": None, "grasped": False},
        {"id": 1, "tool_pos": [0.15, 0.25, 0.05], "color": "blue", "base_pos": None, "grasped": False},
        {"id": 2, "tool_pos": [0.2, 0.3, 0.05], "color": "yellow", "base_pos": None, "grasped": False},
        {"id": 3, "tool_pos": [0.25, 0.35, 0.05], "color": "blue", "base_pos": None, "grasped": False},
    ]
    
    command = {"color": "yellow", "num": 2, "other_num": 1}
    
    initial_state = initialize_state(
        command=command,
        detected_blocks=test_blocks,
        current_pose=[0.0, 0.0, 0.3, 0.0, 0.0, 0.0],
        robot_interface=None
    )
    
    print("✅ Initial state created")
    print(f"   Command: {command}")
    print(f"   Detected {len(test_blocks)} blocks")
    
    result = agent.run_with_state(initial_state)
    
    print("✅ Agent execution completed")
    print(f"   Status: {result['status']}")
    print(f"   Grasped: {result['grasped_count']}/{result['total_to_grasp']}")
    
    if result["status"].value == "completed":
        print("🎉 SUCCESS: Agent completed two-color block grasping task!")
    else:
        print(f"⚠️  Task failed: {result.get('error_message', 'Unknown error')}")

if __name__ == "__main__":
    test_agent()