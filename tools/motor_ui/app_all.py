import threading, time, serial
from flask import Flask, render_template_string
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

PORT = '/dev/ttyUSB0'
BAUD = 115200

# id -> (isim, tip). tip 'S'=ST3020 (0-4095, 360°), 'A'=AX-12A (0-1023, 300°)
MOTORS = [
    # Bacak 1 (Left-Back)   : coxa 3, femur 2, tibia 1
    (1,  'B1 Tibia', 'A'), (2,  'B1 Femur', 'S'), (3,  'B1 Coxa', 'A'),
    # Bacak 2 (Left-Middle) : coxa 6, femur 5, tibia 4
    (4,  'B2 Tibia', 'A'), (5,  'B2 Femur', 'S'), (6,  'B2 Coxa', 'A'),
    # Bacak 3 (Left-Front)  : coxa 9, femur 8, tibia 7
    (7,  'B3 Tibia', 'A'), (8,  'B3 Femur', 'S'), (9,  'B3 Coxa', 'A'),
    # Bacak 4 (Right-Front) : coxa 10, femur 11, tibia 12
    (10, 'B4 Coxa',  'A'), (11, 'B4 Femur', 'S'), (12, 'B4 Tibia', 'A'),
    # Bacak 5 (Right-Middle): coxa 13, femur 14, tibia 15
    (13, 'B5 Coxa',  'A'), (14, 'B5 Femur', 'S'), (15, 'B5 Tibia', 'A'),
    # Bacak 6 (Right-Back)  : coxa 16, femur 17, tibia 18
    (16, 'B6 Coxa',  'A'), (17, 'B6 Femur', 'S'), (18, 'B6 Tibia', 'A'),
]

POS_MAX = {'S': 4095, 'A': 1023}
DEG_SPAN = {'S': 360.0, 'A': 300.0}

MOTOR_IDS = [m[0] for m in MOTORS]
META = {m[0]: {'name': m[1], 'tip': m[2]} for m in MOTORS}

ser      = None
ser_lock = threading.Lock()
state    = {mid: {'pos': -1, 'torque': False, 'min': None, 'max': None,
                  'target': None, 'name': META[mid]['name'], 'tip': META[mid]['tip'],
                  'posmax': POS_MAX[META[mid]['tip']], 'degspan': DEG_SPAN[META[mid]['tip']]}
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
                deadline = time.time() + 3.0
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
        time.sleep(0.5)

# ── Routes ───────────────────────────────────────────────────────────────────
import os
HTML = open(os.path.join(os.path.dirname(__file__), 'templates', 'index_all.html')).read()

@app.route('/')
def index():
    return render_template_string(HTML)

@socketio.on('connect')
def on_connect():
    emit('update', state)

@socketio.on('move')
def on_move(data):
    mid = int(data['id'])
    pmax = state[mid]['posmax']
    pos = max(0, min(pmax, int(data['pos'])))
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
        time.sleep(4.0)          # 18 motor × ~200ms
        ser.reset_input_buffer()
    emit('toast', {'msg': 'EEPROM/açı limitleri kaldırıldı (18 motor).'}, broadcast=True)

if __name__ == '__main__':
    open_serial()
    threading.Thread(target=poll_loop, daemon=True).start()
    print("→ http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
