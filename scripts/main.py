import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from fastapi_app.server import start_fastapi_app
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
        fastapi_thread = threading.Thread(target=start_fastapi_app)
        fastapi_thread.start()

        asyncio_thread = threading.Thread(target=start_asyncio_loop)
        asyncio_thread.start()

        fastapi_thread.join()
        asyncio_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        turtle_controller.destroy_node()
        rclpy.shutdown()
