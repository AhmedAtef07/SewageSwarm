from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import serial

app = Flask(__name__, static_folder='assets')
socketio = SocketIO(app, ping_timeout=5400)


@app.route('/')
def index():
  return render_template('index.html')

# has_any_conn = False

@socketio.on('new-connection')
def test_message():
  print '######### NEW USER CONNECTED #########'
  # if has_any_conn:
    # return
  # has_any_conn = True
  emit('readings', 'New User Connected')

  ser = serial.Serial('/dev/ttyUSB0')
  while True:
    try:
      reading = ser.readline()
      emit('readings', reading, broadcast=True)
    except Exception as e:
      print '############## Some error happened but we suppressed it! ha!', e



if __name__ == '__main__':
  socketio.run(app)
