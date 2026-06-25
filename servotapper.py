import serial
import serial.tools.list_ports
import threading
import webbrowser
import os
import time
from flask import Flask, render_template_string, request

# ==========================
# CONFIG & AUTO-DETECTION
# ==========================
def detect_serial_port():
    # Detect USB serial / ACM ports typically used by Arduino Nano on Pi 5 / Linux / Windows
    ports = [p.device for p in serial.tools.list_ports.comports()
             if "usb" in p.device.lower() or "ttyusb" in p.device.lower() or "ttyacm" in p.device.lower()]
    if ports:
        return ports[0]
    # Fallback defaults
    if os.name == 'posix':
        return "/dev/ttyACM0"
    return "COM3"

SERIAL_PORT = detect_serial_port()
BAUDRATE = 115200

ser = None
simulation_mode = False

try:
    # Use 50ms read timeout to prevent blocking during serial operations
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.05)
    print(f"Successfully connected to serial port {SERIAL_PORT}")
except Exception as e:
    simulation_mode = True
    print(f"Warning: Could not open serial port {SERIAL_PORT}: {e}")
    print("Running in Simulation Mode.")

# In-memory target states
angles = [90, 90, 90, 90, 90, 90]
target_values = [90, 90, 90, 90, 90, 90]
latest_analogs = "0,0,0,0,0,0"

state_lock = threading.Lock()

# ==========================
# SERIAL WRITER THREAD (Throttled & Non-Blocking)
# ==========================
def serial_writer_loop():
    global target_values
    last_sent_cmd = ""

    while True:
        if simulation_mode:
            time.sleep(0.02)
            continue

        with state_lock:
            current_targets = list(target_values)

        cmd = ",".join(str(x) for x in current_targets) + "\n"

        # Only send if values changed
        if cmd != last_sent_cmd:
            if ser and ser.is_open:
                try:
                    ser.write(cmd.encode())
                    ser.flush()
                    last_sent_cmd = cmd
                except Exception as e:
                    print(f"Error writing to serial: {e}")
                    time.sleep(0.5) # Wait a bit on error before retrying
        
        # Max update rate 50Hz (every 20ms) to prevent congestion
        time.sleep(0.02)

threading.Thread(target=serial_writer_loop, daemon=True).start()

# ==========================
# SERIAL READER
# ==========================
def serial_reader():
    global latest_analogs

    if simulation_mode:
        import random
        while True:
            # Generate simulated fluctuating analog values
            vals = [int(512 + 100 * random.uniform(-1, 1)) for _ in range(6)]
            latest_analogs = ",".join(str(v) for v in vals)
            time.sleep(0.5)
        return

    while True:
        try:
            line = ser.readline().decode().strip()
            if line:
                latest_analogs = line
        except Exception as e:
            time.sleep(0.01) # Sleep on read error to prevent CPU starvation

threading.Thread(target=serial_reader, daemon=True).start()

# ==========================
# WEB APP
# ==========================
app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
<title>RoboDog 6-Servo Controller</title>
<link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700&display=swap" rel="stylesheet">
<style>
:root {
    --bg-color: #0b0f19;
    --card-bg: rgba(22, 28, 45, 0.6);
    --border-color: rgba(255, 255, 255, 0.08);
    --text-primary: #f3f4f6;
    --text-secondary: #9ca3af;
    --accent-color: #3b82f6;
    --accent-glow: rgba(59, 130, 246, 0.15);
    --success-color: #10b981;
    --success-bg: rgba(16, 185, 129, 0.1);
    --warning-color: #f59e0b;
    --warning-bg: rgba(245, 158, 11, 0.1);
}

body {
    font-family: 'Plus Jakarta Sans', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    background-color: var(--bg-color);
    background-image: 
        radial-gradient(at 0% 0%, rgba(59, 130, 246, 0.1) 0px, transparent 50%),
        radial-gradient(at 100% 100%, rgba(16, 185, 129, 0.05) 0px, transparent 50%);
    color: var(--text-primary);
    margin: 0;
    padding: 40px 20px;
    min-height: 100vh;
    display: flex;
    justify-content: center;
}

.container {
    max-width: 800px;
    width: 100%;
}

header {
    margin-bottom: 30px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    flex-wrap: wrap;
    gap: 15px;
}

h1 {
    font-size: 28px;
    font-weight: 700;
    margin: 0;
    background: linear-gradient(135deg, #fff 0%, #9ca3af 100%);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
}

.status-badge {
    display: inline-flex;
    align-items: center;
    gap: 8px;
    font-size: 14px;
    font-weight: 600;
    padding: 8px 16px;
    border-radius: 9999px;
    backdrop-filter: blur(10px);
}

.status-connected {
    color: var(--success-color);
    background-color: var(--success-bg);
    border: 1px solid rgba(16, 185, 129, 0.2);
}

.status-simulation {
    color: var(--warning-color);
    background-color: var(--warning-bg);
    border: 1px solid rgba(245, 158, 11, 0.2);
}

.status-dot {
    width: 8px;
    height: 8px;
    border-radius: 50%;
    background-color: currentColor;
    box-shadow: 0 0 8px currentColor;
}

.panel {
    background: var(--card-bg);
    border: 1px solid var(--border-color);
    border-radius: 16px;
    padding: 24px;
    backdrop-filter: blur(20px);
    margin-bottom: 24px;
    box-shadow: 0 4px 30px rgba(0, 0, 0, 0.4);
}

.panel-title {
    font-size: 18px;
    font-weight: 600;
    margin-top: 0;
    margin-bottom: 20px;
    color: #fff;
    display: flex;
    align-items: center;
    gap: 10px;
}

.grid {
    display: flex;
    flex-direction: column;
    gap: 16px;
}

.row {
    display: grid;
    grid-template-columns: 100px 1fr 60px 160px;
    align-items: center;
    gap: 20px;
    padding: 12px 16px;
    background: rgba(255, 255, 255, 0.02);
    border: 1px solid rgba(255, 255, 255, 0.03);
    border-radius: 12px;
    transition: all 0.2s ease;
}

.row:hover {
    background: rgba(255, 255, 255, 0.04);
    border-color: rgba(255, 255, 255, 0.06);
}

.row.disabled {
    opacity: 0.5;
}

.label {
    font-weight: 600;
    font-size: 14px;
    color: var(--text-secondary);
}

.slider-container {
    display: flex;
    align-items: center;
}

.slider {
    -webkit-appearance: none;
    width: 100%;
    height: 6px;
    border-radius: 9999px;
    background: rgba(255, 255, 255, 0.1);
    outline: none;
    transition: background 0.2s;
}

.slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 18px;
    height: 18px;
    border-radius: 50%;
    background: var(--accent-color);
    box-shadow: 0 0 10px var(--accent-glow), 0 0 0 4px rgba(59, 130, 246, 0.2);
    cursor: pointer;
    transition: transform 0.1s, background-color 0.2s;
}

.slider::-webkit-slider-thumb:hover {
    transform: scale(1.2);
    background: #60a5fa;
}

.slider:disabled::-webkit-slider-thumb {
    background: var(--text-secondary);
    box-shadow: none;
    cursor: not-allowed;
    transform: none;
}

.value-display {
    font-family: monospace;
    font-size: 16px;
    font-weight: 700;
    text-align: right;
    color: var(--accent-color);
}

.btn-group {
    display: flex;
    gap: 8px;
}

button {
    flex: 1;
    font-family: inherit;
    font-size: 13px;
    font-weight: 600;
    padding: 8px 12px;
    border-radius: 8px;
    border: 1px solid rgba(255, 255, 255, 0.08);
    background: rgba(255, 255, 255, 0.05);
    color: var(--text-primary);
    cursor: pointer;
    transition: all 0.2s ease;
}

button:hover {
    background: rgba(255, 255, 255, 0.1);
    border-color: rgba(255, 255, 255, 0.15);
}

button.btn-accent {
    background: var(--accent-color);
    border-color: transparent;
    color: white;
}

button.btn-accent:hover {
    background: #2563eb;
    box-shadow: 0 0 12px rgba(59, 130, 246, 0.4);
}

.analog-display {
    font-family: monospace;
    font-size: 15px;
    background: rgba(0, 0, 0, 0.3);
    padding: 12px 16px;
    border-radius: 8px;
    border: 1px solid var(--border-color);
    color: var(--success-color);
    letter-spacing: 1px;
}
</style>
</head>
<body>

<div class="container">
    <header>
        <h1>RoboDog Servo Panel</h1>
        <div class="status-badge {% if simulation_mode %}status-simulation{% else %}status-connected{% endif %}">
            <span class="status-dot"></span>
            Status: {% if simulation_mode %}Simulation ({{port}} offline){% else %}Connected to {{port}}{% endif %}
        </div>
    </header>

    <!-- MASTER CONTROLS -->
    <div class="panel" style="border-left: 4px solid var(--accent-color);">
        <h3 class="panel-title">
            <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" style="color: var(--accent-color);"><path d="M12.22 2h-.44a2 2 0 0 0-2 2v.18a2 2 0 0 1-1 1.73l-.43.25a2 2 0 0 1-2 0l-.15-.08a2 2 0 0 0-2.73.73l-.22.38a2 2 0 0 0 .73 2.73l.15.1a2 2 0 0 1 1 1.72v.51a2 2 0 0 1-1 1.74l-.15.09a2 2 0 0 0-.73 2.73l.22.38a2 2 0 0 0 2.73.73l.15-.08a2 2 0 0 1 2 0l.43.25a2 2 0 0 1 1 1.73V20a2 2 0 0 0 2 2h.44a2 2 0 0 0 2-2v-.18a2 2 0 0 1 1-1.73l.43-.25a2 2 0 0 1 2 0l.15.08a2 2 0 0 0 2.73-.73l.22-.39a2 2 0 0 0-.73-2.73l-.15-.08a2 2 0 0 1-1-1.74v-.5a2 2 0 0 1 1-1.74l.15-.1a2 2 0 0 0 .73-2.73l-.22-.38a2 2 0 0 0-2.73-.73l-.15.08a2 2 0 0 1-2 0l-.43-.25a2 2 0 0 1-1-1.73V4a2 2 0 0 0-2-2z"></path><circle cx="12" cy="12" r="3"></circle></svg>
            Master Controller
        </h3>
        <div class="row" style="background: rgba(59, 130, 246, 0.03); border-color: rgba(59, 130, 246, 0.1);">
            <div class="label" style="color: #fff;">All Servos</div>
            <div class="slider-container">
                <input class="slider"
                       type="range"
                       min="0"
                       max="180"
                       value="90"
                       id="master-slider"
                       oninput="updateAllValues(this.value)">
            </div>
            <div class="value-display" id="master-val">90</div>
            <div class="btn-group">
                <button onclick="detachAllServos()">Detach All</button>
                <button onclick="attachAllServos()" class="btn-accent">Attach All</button>
            </div>
        </div>
    </div>

    <!-- INDIVIDUAL SERVOS -->
    <div class="panel">
        <h3 class="panel-title">Individual Joints</h3>
        <div class="grid">
            {% for i in range(6) %}
            <div class="row" id="row{{i}}">
                <div class="label">Servo {{i+1}}</div>
                <div class="slider-container">
                    <input class="slider"
                           type="range"
                           min="0"
                           max="180"
                           value="{{angles[i]}}"
                           id="s{{i}}"
                           oninput="updateValue({{i}})">
                </div>
                <div class="value-display" id="v{{i}}">{{angles[i]}}</div>
                <div class="btn-group">
                    <button onclick="detachServo({{i}})" id="det{{i}}">Detach</button>
                    <button onclick="attachServo({{i}})" id="att{{i}}" style="display:none;" class="btn-accent">Attach</button>
                </div>
            </div>
            {% endfor %}
        </div>
    </div>

    <!-- ANALOGS -->
    <div class="panel">
        <h3 class="panel-title">Analog Telemetry</h3>
        <div class="analog-display" id="analogs">Telemetry Loading...</div>
    </div>
</div>

<script>
let attached = [true, true, true, true, true, true];

function getAngles() {
    let values = [];
    for(let i=0; i<6; i++) {
        if (!attached[i]) {
            values.push(404);
        } else {
            values.push(parseInt(document.getElementById("s"+i).value));
        }
    }
    return values;
}

function send() {
    fetch("/send", {
        method: "POST",
        headers: {
            "Content-Type": "application/json"
        },
        body: JSON.stringify({
            values: getAngles()
        })
    });
}

function updateValue(i) {
    document.getElementById("v"+i).innerText = document.getElementById("s"+i).value;
    send();
}

function updateAllValues(val) {
    document.getElementById("master-val").innerText = val;
    for(let i=0; i<6; i++) {
        if (attached[i]) {
            document.getElementById("s"+i).value = val;
            document.getElementById("v"+i).innerText = val;
        }
    }
    send();
}

function detachServo(i) {
    attached[i] = false;
    document.getElementById("s"+i).disabled = true;
    document.getElementById("row"+i).classList.add("disabled");
    document.getElementById("det"+i).style.display = "none";
    document.getElementById("att"+i).style.display = "inline-block";
    send();
}

function attachServo(i) {
    attached[i] = true;
    document.getElementById("s"+i).disabled = false;
    document.getElementById("row"+i).classList.remove("disabled");
    document.getElementById("det"+i).style.display = "inline-block";
    document.getElementById("att"+i).style.display = "none";
    send();
}

function detachAllServos() {
    for(let i=0; i<6; i++) {
        detachServo(i);
    }
}

function attachAllServos() {
    let val = document.getElementById("master-slider").value;
    for(let i=0; i<6; i++) {
        attachServo(i);
        document.getElementById("s"+i).value = val;
        document.getElementById("v"+i).innerText = val;
    }
    send();
}

setInterval(() => {
    fetch("/analogs")
    .then(r => r.text())
    .then(t => {
        document.getElementById("analogs").innerText = t;
    });
}, 100);
</script>

</body>
</html>
"""

@app.route("/")
def home():
    return render_template_string(HTML, angles=angles, simulation_mode=simulation_mode, port=SERIAL_PORT)

@app.route("/analogs")
def analogs():
    return latest_analogs

@app.route("/send", methods=["POST"])
def send():
    global target_values, angles

    data = request.json
    values = data["values"]

    with state_lock:
        target_values = list(values)
        for i in range(6):
            if 0 <= values[i] <= 180:
                angles[i] = values[i]

    return "OK"

# ==========================
# START
# ==========================
if __name__ == "__main__":
    webbrowser.open("http://127.0.0.1:5000")
    app.run(host="0.0.0.0", port=5000, threaded=True)