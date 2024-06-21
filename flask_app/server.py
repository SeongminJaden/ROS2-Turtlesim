from . import app

def start_flask_app():
    app.run(host='0.0.0.0', port=4000)
