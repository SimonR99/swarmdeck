"""Container-side payload probe for test_dds_transport_docker.py."""

import json
from pathlib import Path
import sys
import time

import rclpy
from sensor_msgs.msg import Image


def loopback_bytes():
    line = next(
        line for line in Path("/proc/net/dev").read_text().splitlines() if "lo:" in line
    )
    return int(line.split(":")[1].split()[8])


def main():
    role = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node(f"dds_transport_{role}")
    try:
        if role == "reader":
            received = []
            node.create_subscription(
                Image,
                "/dds_smoke/image",
                lambda msg: received.append(len(msg.data)),
                10,
            )
            deadline = time.monotonic() + 25
            while len(received) < 50 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            print(
                json.dumps({"received": len(received), "sizes": sorted(set(received))}),
                flush=True,
            )
            assert len(received) == 50
        else:
            pub = node.create_publisher(Image, "/dds_smoke/image", 10)
            deadline = time.monotonic() + 25
            while pub.get_subscription_count() < 1 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            assert pub.get_subscription_count() == 1
            msg = Image(
                height=1024,
                width=1024,
                encoding="mono8",
                step=1024,
                data=bytes(1024 * 1024),
            )
            before = loopback_bytes()
            for _ in range(50):
                pub.publish(msg)
                time.sleep(0.1)
            time.sleep(2)
            print(
                json.dumps(
                    {
                        "sent": 50,
                        "loopback_tx_bytes": loopback_bytes() - before,
                        "shm_sizes": sorted(
                            {
                                p.stat().st_size
                                for p in Path("/dev/shm").glob("fastrtps_*")
                            }
                        ),
                    }
                ),
                flush=True,
            )
            time.sleep(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
