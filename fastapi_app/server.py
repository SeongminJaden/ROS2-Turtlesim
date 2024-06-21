import uvicorn
from . import app

def start_fastapi_app():
    uvicorn.run(app, host="0.0.0.0", port=4000)
