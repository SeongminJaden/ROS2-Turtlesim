from fastapi import APIRouter, Request
from pydantic import BaseModel
from . import app
from ros2 import turtle_controller
import asyncio

router = APIRouter()

class MoveRequest(BaseModel):
    x: float
    y: float

class ObstacleRequest(BaseModel):
    x1: float
    y1: float
    x2: float
    y2: float

@router.post("/move")
async def move(request: MoveRequest):
    try:
        target_x = request.x
        target_y = request.y

        loop = asyncio.get_event_loop()
        await loop.run_in_executor(None, asyncio.run, turtle_controller.navigate_to(target_x, target_y))

        return {"message": "Move command sent successfully"}
    except Exception as e:
        return {"message": f"Internal Server Error: {e}"}

@router.post("/add_obstacle")
async def add_obstacle(request: ObstacleRequest):
    try:
        x1 = request.x1
        y1 = request.y1
        x2 = request.x2
        y2 = request.y2

        turtle_controller.add_obstacle(x1, y1, x2, y2)
        return {"message": "Obstacle added successfully"}
    except Exception as e:
        return {"message": f"Internal Server Error: {e}"}

app.include_router(router)
