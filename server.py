from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial

app = Flask(__name__, static_folder='assets')
socketio = SocketIO(app)


@app.route('/')
def index():
  return render_template('index.html')


@socketio.on('new-connection')
def test_message():
  print '######### NEW USER CONNECTED #########'
  emit('readings', 'New User Connected')

  ser = serial.Serial('/dev/ttyUSB0')
  while True:
    reading = ser.readline()
    emit('readings', reading)


if __name__ == '__main__':
  socketio.run(app)
