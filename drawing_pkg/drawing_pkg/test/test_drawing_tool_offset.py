from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_source(name):
    return (ROOT / "src" / name).read_text(encoding="utf-8")


def function_body(source, signature):
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for i in range(brace, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:i]
    raise AssertionError(f"Could not parse body for {signature}")


def test_follower_tracks_and_publishes_tool_point_not_base_center():
    source = read_source("drawing_follower_node.cpp")
    update_pose = function_body(source, "bool updateRobotPose")
    distance_fn = function_body(source, "double distanceToRobot")

    assert "updateToolPoint();" in update_pose
    assert "pose.pose.position.x = tool_x_;" in update_pose
    assert "pose.pose.position.y = tool_y_;" in update_pose
    assert "p.x - tool_x_" in distance_fn
    assert "p.y - tool_y_" in distance_fn


def test_follower_uses_off_center_control_law_for_cmd_vel():
    source = read_source("drawing_follower_node.cpp")
    command = function_body(source, "void publishCommandToTarget")

    assert "use_tool_offset_control_" in command
    assert "tool_offset_x_" in command
    assert "(-s * ux + c * uy) / a" in command
    assert "cmd.linear.x = clamp(v, -max_reverse_speed_, linear_speed_)" in command


def test_navigator_offsets_base_goal_from_drawing_goal():
    source = read_source("drawing_navigator_node.cpp")
    go_callback = function_body(source, "void goCallback")
    offset_goal = function_body(source, "void applyToolOffsetToGoal")

    assert "computeToolPoint" in go_callback
    assert "applyToolOffsetToGoal();" in go_callback
    assert "drawing_goal_point_.x - c * tool_offset_x_" in offset_goal
    assert "drawing_goal_point_.y - s * tool_offset_x_" in offset_goal


def test_follower_and_navigator_share_tool_offset_params():
    follower_params = (ROOT / "config" / "drawing_follower_params.yaml").read_text(encoding="utf-8")
    navigator_params = (ROOT / "config" / "drawing_navigator_params.yaml").read_text(encoding="utf-8")

    assert "use_tool_offset_control: true" in follower_params
    assert "stop_at_corners: false" in follower_params
    assert "tool_offset_x: 0.07" in follower_params
    assert "tool_offset_x: 0.07" in navigator_params
    assert "max_reverse_speed: 0.018" in follower_params


def test_closed_paths_choose_start_by_distance_and_heading():
    source = read_source("drawing_follower_node.cpp")
    rotate = function_body(source, "void rotateClosedPathToNearestPoint")

    assert "findBestClosedPathStart" in rotate
    assert "reverse_selected" in rotate
    assert "std::reverse" in rotate


def test_closed_paths_have_explicit_final_closure_phase():
    source = read_source("drawing_follower_node.cpp")
    control_step = function_body(source, "void controlStep()")

    assert "handleClosedPathFinish()" in control_step
    assert "closed_goal_tolerance_" in source
