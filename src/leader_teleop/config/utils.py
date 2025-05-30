import subprocess
import os
import yaml

VALID_CAPS = {"MJPG", "YUY2", "YUYV"}  # allowed in config

CONFIG_PATH = os.path.join(os.path.dirname(__file__), "robot_config.yaml")


def get_dev_paths(prefix):
    return [f"/dev/{f}" for f in os.listdir("/dev") if f.startswith(prefix)]


def get_serial_for_device(dev_path):
    try:
        result = subprocess.run(
            ["udevadm", "info", "--query=property", "--name", dev_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
            text=True,
        )
        for line in result.stdout.splitlines():
            if line.startswith("ID_SERIAL="):
                return line.split("=", 1)[1]
    except subprocess.CalledProcessError:
        pass
    return None


def supports_valid_video_format(dev_path):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", dev_path, "--list-formats-ext"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
            text=True,
        )
        return any(fmt in result.stdout for fmt in VALID_CAPS)
    except subprocess.CalledProcessError:
        return False


def find_serial_to_device_map(dev_prefix, filter_func=None):
    mapping = {}
    for dev in get_dev_paths(dev_prefix):
        if filter_func and not filter_func(dev):
            continue
        serial = get_serial_for_device(dev)
        if serial:
            mapping[serial] = dev
    return mapping


# ──────────────────────────────────────────────────────────────
# Helper: detect caps for a /dev/videoX node
# ──────────────────────────────────────────────────────────────
def detect_camera_caps(dev_path: str) -> str:
    """
    Inspect a v4l2 device and return "MJPG" or "YUY2".
    If none of those are advertised, return "UNKNOWN".
    """
    try:
        result = subprocess.run(
            ["v4l2-ctl", "--device", dev_path, "--list-formats-ext"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
            text=True,
        ).stdout.upper()  # capitalise for easy find()
    except subprocess.CalledProcessError:
        return "UNKNOWN"

    if "MJPG" in result:
        return "MJPG"
    if "YUYV" in result:  # YUYV is the same four-character code
        return "YUY2"
    return "UNKNOWN"


# ──────────────────────────────────────────────────────────────
def load_config_and_resolve_devices(config_path):
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    resolved = {
        "dxl_device": config["devices"].get("dxl_device"),
        "rs485_device": config["devices"].get("rs485_device"),
        "cameras": {"scene": {}, "wrist": {}},
    }

    for cam_type in ["scene", "wrist"]:
        for cam in config["devices"]["cameras"].get(cam_type, []):
            alias = cam.get("alias")
            devnode = cam.get("device", "NOT_FOUND")
            width = cam.get("width", 640)
            height = cam.get("height", 480)

            caps = detect_camera_caps(devnode) if devnode != "NOT_FOUND" else "UNKNOWN"
            if caps not in VALID_CAPS:
                print(
                    f"❌ Unable to detect supported caps for '{alias}' "
                    f"({devnode}).  Got '{caps}'. Expected one of {sorted(VALID_CAPS)}"
                )

            resolved["cameras"][cam_type][alias] = {
                "alias": alias,
                "device": devnode,
                "width": width,
                "height": height,
                "caps": caps,
            }

    return resolved


device_config = load_config_and_resolve_devices(CONFIG_PATH)
config = yaml.safe_load(open(CONFIG_PATH, "r"))

# ────────────────────────────────────────────────────────────
# Example usage
# ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    resolved = device_config

    print("Dynamixel Device:", resolved["dxl_device"])
    print("RS485 Device:", resolved["rs485_device"])
    print("\nCameras:")
    for group, cams in resolved["cameras"].items():
        print(f"  {group}:")
        for cam in cams.values():
            print(
                f"    - {cam['alias']}  "
                f"dev={cam['device']}  "
                f"{cam['width']}x{cam['height']}  "
                f"caps={cam['caps']}"
            )
