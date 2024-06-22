from .turtle_controller import TurtleController
import rclpy

rclpy.init(args=None)
turtle_controller = TurtleController()

def get_turtle_controller():
    return turtle_controller
