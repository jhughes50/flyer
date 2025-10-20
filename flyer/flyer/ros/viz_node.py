""" 
    Jason Hughes
    January 2025

    Script to start localization node
"""

import rclpy
import threading

from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from flyer.viz.map import VizNode

def main(args=None) -> None:
    rclpy.init(args=args)
    
    node = VizNode()
    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
