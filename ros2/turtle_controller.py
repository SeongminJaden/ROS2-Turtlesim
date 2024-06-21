import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import asyncio
import socket
import json

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.obstacles = []
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.current_x = self.initial_x
        self.current_y = self.initial_y
        self.theta = 0.0  # 로봇의 헤딩(방향) 값, 라디안 단위
        self.udp_ip = "192.168.1.5"  # UDP 서버의 IP 주소
        self.udp_port = 5000  # UDP 서버의 포트
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def move_turtle(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x * 2.0  # 선형 속도 두 배 증가
        twist.angular.z = angular_z * 2.0  # 각속도 두 배 증가
        self.publisher_.publish(twist)
        print(f"Moving turtle: linear_x={linear_x}, angular_z={angular_z}")

    def add_obstacle(self, x1, y1, x2, y2):
        self.obstacles.append((x1, y1, x2, y2))
        print(f"Obstacle added: ({x1}, {y1}) to ({x2}, {y2})")

    def remove_obstacle(self, index):
        if 0 <= index < len(self.obstacles):
            removed = self.obstacles.pop(index)
            print(f"Obstacle removed: {removed}")
            return True
        return False

    def is_collision(self, x, y):
        for (x1, y1, x2, y2) in self.obstacles:
            if x1 <= x <= x2 and y1 <= y <= y2:
                print(f"Collision detected at: ({x}, {y}) with obstacle: ({x1}, {y1}) to ({x2}, {y2})")
                return True
        return False

    async def navigate_to(self, target_x, target_y):
        try:
            step_size = 0.1  # 스텝 사이즈 조정

            while abs(self.current_x - target_x) > 0.1 or abs(self.current_y - target_y) > 0.1:
                angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)

                # 목표 지점을 향해 회전
                while abs(self.theta - angle_to_target) > 0.1:
                    angular_speed = 1.0 if angle_to_target > self.theta else -1.0  # 각속도 조정
                    self.move_turtle(0.0, angular_speed)
                    self.theta += angular_speed * 0.1  # 현재 각도 갱신
                    await asyncio.sleep(0.1)
                
                # 목표 지점으로 이동
                next_x = self.current_x + step_size * math.cos(angle_to_target)
                next_y = self.current_y + step_size * math.sin(angle_to_target)

                if not self.is_collision(next_x, next_y):
                    self.current_x = next_x
                    self.current_y = next_y
                    self.move_turtle(0.5, 0.0)  # 선형 속도 조정
                else:
                    # 장애물 회피 로직
                    await self.avoid_obstacle()
                
                rclpy.spin_once(self, timeout_sec=0.1)
                await asyncio.sleep(0.1)

            # 목표 지점에 도달하면 로봇 멈추기
            self.move_turtle(0.0, 0.0)
            print(f"Reached target: ({target_x}, {target_y})")
        except Exception as e:
            print(f"Error in navigate_to: {e}")
            raise e

    async def avoid_obstacle(self):
        try:
            # 장애물을 회피하기 위해 일정한 각도로 회전 후 직진
            initial_theta = self.theta
            turn_angle = math.pi / 8  # 22.5도 회전
            while abs(self.theta - initial_theta) < turn_angle:
                self.move_turtle(0.0, 1.0)  # 각속도 조정
                self.theta += 1.0 * 0.1
                await asyncio.sleep(0.1)
            
            # 일정 시간 동안 직진하여 장애물 회피
            for _ in range(10):  # 장애물 회피 지속 시간 증가
                self.move_turtle(0.5, 0.0)  # 선형 속도 조정
                self.current_x += 0.5 * 0.1 * math.cos(self.theta)
                self.current_y += 0.5 * 0.1 * math.sin(self.theta)
                await asyncio.sleep(0.1)
            print(f"Obstacle avoided, new position: ({self.current_x}, {self.current_y})")
        except Exception as e:
            print(f"Error in avoid_obstacle: {e}")
            raise e

    async def send_coordinates(self):
        while True:
            coordinates = {
                'x': self.current_x,
                'y': self.current_y,
                'theta': self.theta
            }
            self.sock.sendto(json.dumps(coordinates).encode(), (self.udp_ip, self.udp_port))
            print(f'Sent coordinates: {coordinates}')
            await asyncio.sleep(1)  # 1초마다 좌표 전송
