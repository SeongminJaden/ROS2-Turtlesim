from flask import request, jsonify
from . import app
from ros2 import turtle_controller
import asyncio

@app.route('/move', methods=['POST'])
def move():
    try:
        data = request.get_json()
        if not data or 'x' not in data or 'y' not in data:
            return jsonify({'message': 'Invalid JSON'}), 400

        target_x = data['x']
        target_y = data['y']

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(turtle_controller.navigate_to(target_x, target_y))
        loop.close()
        
        return jsonify({'message': 'Move command sent successfully'}), 200
    except Exception as e:
        print(f'Error handling /move request: {e}')
        return jsonify({'message': 'Internal Server Error'}), 500

@app.route('/add_obstacle', methods=['POST'])
def add_obstacle():
    try:
        data = request.get_json()
        if not data or 'x1' not in data or 'y1' not in data or 'x2' not in data or 'y2' not in data:
            return jsonify({'message': 'Invalid JSON'}), 400

        x1 = data['x1']
        y1 = data['y1']
        x2 = data['x2']
        y2 = data['y2']

        turtle_controller.add_obstacle(x1, y1, x2, y2)
        return jsonify({'message': 'Obstacle added successfully'}), 200
    except Exception as e:
        print(f'Error handling /add_obstacle request: {e}')
        return jsonify({'message': 'Internal Server Error'}), 500
