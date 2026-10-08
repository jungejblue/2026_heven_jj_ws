"""Read-only ROS graph checks for competing ego control sources."""

CONTROL_TOPIC = "/carla/ego_vehicle/vehicle_control_cmd"
MANUAL_TOPIC = "/carla/ego_vehicle/vehicle_control_cmd_manual"
AUTOPILOT_TOPIC = "/carla/ego_vehicle/enable_autopilot"


def publisher_names(node, topic):
    return [
        f"{info.node_namespace.rstrip('/')}/{info.node_name}"
        for info in node.get_publishers_info_by_topic(topic)
    ]


def warn_competing_sources(node, mode):
    control = publisher_names(node, CONTROL_TOPIC)
    manual = publisher_names(node, MANUAL_TOPIC)
    auto = publisher_names(node, AUTOPILOT_TOPIC)
    warning = None
    if mode == "wasd" and (len(control) > 1 or manual):
        warning = f"Competing vehicle command publishers: {control + manual}"
    if mode == "autopilot" and (control or manual):
        warning = f"Vehicle command publishers while autopilot selected: {control + manual}"
    if mode == "external" and len(control) + len(manual) > 1:
        warning = f"Multiple external ego control publishers: {control + manual}"
    if mode == "external" and auto and (control or manual):
        warning = f"Autopilot and external command sources coexist: {auto + control + manual}"
    if warning != getattr(node, "_last_control_warning", None):
        if warning:
            node.get_logger().warning(warning)
        node._last_control_warning = warning
    return control, manual, auto
