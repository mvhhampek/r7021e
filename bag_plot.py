import matplotlib.pyplot as plt
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


BAG_DIR = "/rosbag2_2025_09_17-17_08_54"
ODOM_TOPIC = "/odom"
WPS_TOPIC = "/waypoints"


def pose_xy(p):
    pos = getattr(p, "position", getattr(p, "pose", p).position)
    return pos.x, pos.y


def extract_waypoints_xy(msg):
    xs, ys = [], []
    if hasattr(msg, "poses"):
        for p in msg.poses:
            x, y = pose_xy(p)
            xs.append(x)
            ys.append(y)
    elif hasattr(msg, "waypoints"):
        for p in msg.waypoints:
            x, y = pose_xy(p)
            xs.append(x)
            ys.append(y)
    elif hasattr(msg, "pose"):
        x, y = pose_xy(msg.pose)
        xs.append(x)
        ys.append(y)
    return xs, ys


def main():
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=BAG_DIR, storage_id="mcap"), ConverterOptions("", "")
    )
    topics = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if ODOM_TOPIC not in topics:
        raise SystemExit(f"Missing {ODOM_TOPIC}. Available: {list(topics)}")
    if WPS_TOPIC not in topics:
        raise SystemExit(f"Missing {WPS_TOPIC}. Available: {list(topics)}")
    Odom = get_message(topics[ODOM_TOPIC])
    WpT = get_message(topics[WPS_TOPIC])

    xs_o, ys_o = [], []
    xs_w, ys_w = [], []
    last_wp_serialized = None

    while reader.has_next():
        tp, data, _ = reader.read_next()
        if tp == ODOM_TOPIC:
            msg = deserialize_message(data, Odom)
            xs_o.append(msg.pose.pose.position.x)
            ys_o.append(msg.pose.pose.position.y)
        elif tp == WPS_TOPIC:
            last_wp_serialized = data

    if last_wp_serialized:
        wp_msg = deserialize_message(last_wp_serialized, WpT)
        xs_w, ys_w = extract_waypoints_xy(wp_msg)

    plt.plot(xs_o, ys_o, label="odom")
    if xs_w:
        plt.scatter(xs_w, ys_w, s=20, label="waypoints")
    ax = plt.gca()
    ax.set_aspect("equal", adjustable="box")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
