from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from . import app
from ros2 import get_turtle_controller

router = APIRouter()

class MoveRequest(BaseModel):
    x: float
    y: float

class ObstacleRequest(BaseModel):
    x1: float
    y1: float
    x2: float
    y2: float

class RemoveObstacleRequest(BaseModel):
    index: int

@router.post("/move")
async def move(request: MoveRequest):
    try:
        turtle_controller = get_turtle_controller()
        target_x = request.x
        target_y = request.y

        await turtle_controller.navigate_to(target_x, target_y)
        
        return {"message": "Move command sent successfully"}
    except Exception as e:
        print(f"Error in /move: {e}")
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")

@router.post("/add_obstacle")
async def add_obstacle(request: ObstacleRequest):
    try:
        turtle_controller = get_turtle_controller()
        x1 = request.x1
        y1 = request.y1
        x2 = request.x2
        y2 = request.y2

        turtle_controller.add_obstacle(x1, y1, x2, y2)
        return {"message": "Obstacle added successfully"}
    except Exception as e:
        print(f"Error in /add_obstacle: {e}")
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")

@router.post("/remove_obstacle")
async def remove_obstacle(request: RemoveObstacleRequest):
    try:
        turtle_controller = get_turtle_controller()
        index = request.index

        if turtle_controller.remove_obstacle(index):
            return {"message": "Obstacle removed successfully"}
        else:
            raise HTTPException(status_code=404, detail="Obstacle not found")
    except Exception as e:
        print(f"Error in /remove_obstacle: {e}")
        raise HTTPException(status_code=500, detail=f"Internal Server Error: {e}")

app.include_router(router)
