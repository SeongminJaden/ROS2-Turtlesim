from flask_app.server import start_flask_app
from ros2 import turtle_controller
import threading
import asyncio
import rclpy

def start_asyncio_loop():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.create_task(turtle_controller.send_coordinates())
    loop.run_forever()

if __name__ == '__main__':
    try:
        flask_thread = threading.Thread(target=start_flask_app)
        flask_thread.start()

        asyncio_thread = threading.Thread(target=start_asyncio_loop)
        asyncio_thread.start()

        flask_thread.join()
        asyncio_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()
