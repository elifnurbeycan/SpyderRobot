import threading, time, serial
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

PORT      = '/dev/ttyUSB1'
BAUD      = 115200
MOTOR_IDS = [2, 5, 8, 11, 14, 17]
NAMES     = ['Femur 1','Femur 2','Femur 3','Femur 4','Femur 5','Femur 6']

ser      = None
ser_lock = threading.Lock()
state    = {mid: {'pos': -1, 'torque': False, 'min': None, 'max': None, 'target': 2048}
            for mid in MOTOR_IDS}

# ── Serial ──────────────────────────────────────────────────────────────────
def open_serial():
    global ser
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
        time.sleep(2)
        ser.readline()           # READY satırı
        print(f"Seri açık: {PORT}")
        return True
    except Exception as e:
        print(f"Seri HATA: {e}")
        return False

def send_raw(cmd):
    try:
        ser.reset_input_buffer()
        ser.write((cmd + '\n').encode())
        ser.flush()
    except: pass

def poll_loop():
    while True:
        with ser_lock:
            try:
                ser.reset_input_buffer()
                ser.write(b'A\n')
                ser.flush()
                deadline = time.time() + 2.0
                while time.time() < deadline:
                    if ser.in_waiting:
                        line = ser.readline().decode(errors='ignore').strip()
                        if line.startswith('P '):
                            parts = line.split()
                            if len(parts) == 3:
                                mid, pos = int(parts[1]), int(parts[2])
                                if mid in state and pos >= 0:
                                    state[mid]['pos'] = pos
                        elif line == 'A DONE':
                            break
                    else:
                        time.sleep(0.005)
            except: pass
        socketio.emit('update', state)
        time.sleep(0.4)

# ── Routes ───────────────────────────────────────────────────────────────────
HTML = open('templates/index.html').read()

@app.route('/')
def index():
    return render_template_string(HTML)

@socketio.on('connect')
def on_connect():
    emit('update', state)

@socketio.on('move')
def on_move(data):
    mid, pos = int(data['id']), max(0, min(4095, int(data['pos'])))
    state[mid]['target'] = pos
    with ser_lock:
        send_raw(f'M {mid} {pos}')

@socketio.on('torque')
def on_torque(data):
    mid, on = int(data['id']), bool(data['on'])
    state[mid]['torque'] = on
    with ser_lock:
        send_raw(f'T {mid} {1 if on else 0}')

@socketio.on('set_limit')
def on_set_limit(data):
    mid, typ = int(data['id']), data['type']
    pos = state[mid]['pos']
    if pos >= 0:
        state[mid][typ] = pos
        emit('update', state, broadcast=True)

@socketio.on('clear_limits')
def on_clear(data):
    mid = int(data['id'])
    state[mid]['min'] = state[mid]['max'] = None
    emit('update', state, broadcast=True)

@socketio.on('unlim_all')
def on_unlim_all():
    with ser_lock:
        send_raw('UNLIMALL')
        # Yanıtı bekle (6 motor × ~200ms)
        time.sleep(1.5)
        ser.reset_input_buffer()
    emit('toast', {'msg': 'EEPROM limitleri kaldırıldı! Tüm motorlar 0–4095 aralığında.'}, broadcast=True)

if __name__ == '__main__':
    open_serial()
    threading.Thread(target=poll_loop, daemon=True).start()
    print("→ http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
