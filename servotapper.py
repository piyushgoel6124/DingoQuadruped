import serial
import threading
import webbrowser
from flask import Flask, render_template_string, request

# ==========================
# CONFIG
# ==========================
SERIAL_PORT = "COM3"      # Change to your Nano port
BAUDRATE = 115200

ser = None
simulation_mode = False

try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    print(f"Successfully connected to serial port {SERIAL_PORT}")
except Exception as e:
    simulation_mode = True
    print(f"Warning: Could not open serial port {SERIAL_PORT}: {e}")
    print("Running in Simulation Mode.")

angles = [90, 90, 90, 90, 90, 90]
latest_analogs = "0,0,0,0,0,0"

# ==========================
# SERIAL READER
# ==========================
def serial_reader():
    global latest_analogs

    if simulation_mode:
        import time
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

        except:
            pass

threading.Thread(target=serial_reader, daemon=True).start()

# ==========================
# WEB APP
# ==========================
app = Flask(__name__)

HTML = """
<!DOCTYPE html>
<html>
<head>
<title>6 Servo Controller</title>

<style>
body{
    font-family:Arial;
    margin:30px;
}

.slider{
    width:500px;
}

.row{
    margin-bottom:20px;
}

button{
    padding:10px;
    margin:5px;
}

h2{
    margin-bottom:10px;
}

.status-badge {
    display: inline-block;
    font-size: 14px;
    font-weight: bold;
    padding: 6px 12px;
    border-radius: 4px;
    margin-bottom: 25px;
}

.status-connected {
    color: #2b542c;
    background-color: #dff0d8;
    border: 1px solid #d6e9c6;
}

.status-simulation {
    color: #a94442;
    background-color: #f2dede;
    border: 1px solid #ebccd1;
}
</style>

</head>
<body>

<h2>6 Servo Controller</h2>
<div class="status-badge {% if simulation_mode %}status-simulation{% else %}status-connected{% endif %}">
    Status: {% if simulation_mode %}Simulation Mode ({{port}} unavailable){% else %}Connected to {{port}}{% endif %}
</div>

{% for i in range(6) %}
<div class="row">
Servo {{i+1}}
<input class="slider"
       type="range"
       min="0"
       max="180"
       value="{{angles[i]}}"
       id="s{{i}}"
       oninput="updateValue({{i}})">
<span id="v{{i}}">{{angles[i]}}</span>

<button onclick="detachServo({{i}})">Detach</button>
<button onclick="attachServo({{i}})">Attach</button>
</div>
{% endfor %}

<hr>

<h3>Analog Inputs</h3>
<div id="analogs">Loading...</div>

<script>

let attached = [true, true, true, true, true, true];

function getAngles()
{
    let values=[];

    for(let i=0;i<6;i++)
    {
        if (!attached[i]) {
            values.push(404);
        } else {
            values.push(
                parseInt(document.getElementById("s"+i).value)
            );
        }
    }

    return values;
}

function send()
{
    fetch("/send",
    {
        method:"POST",
        headers:
        {
            "Content-Type":"application/json"
        },
        body:JSON.stringify(
        {
            values:getAngles()
        })
    });
}

function updateValue(i)
{
    document.getElementById("v"+i).innerText=
        document.getElementById("s"+i).value;

    send();
}

function detachServo(i)
{
    attached[i] = false;
    document.getElementById("s"+i).disabled = true;
    send();
}

function attachServo(i)
{
    attached[i] = true;
    document.getElementById("s"+i).disabled = false;
    send();
}

setInterval(()=>{
    fetch("/analogs")
    .then(r=>r.text())
    .then(t=>{
        document.getElementById("analogs").innerText=t;
    });
},100);

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
    global angles

    data = request.json
    values = data["values"]

    cmd = ",".join(str(x) for x in values) + "\n"

    if not simulation_mode and ser:
        try:
            ser.write(cmd.encode())
        except Exception as e:
            print(f"Error writing to serial: {e}")
    else:
        print(f"[SIMULATION] Sending command: {cmd.strip()}")

    for i in range(6):
        if 0 <= values[i] <= 180:
            angles[i] = values[i]

    return "OK"

# ==========================
# START
# ==========================
if __name__ == "__main__":
    webbrowser.open("http://127.0.0.1:5000")
    app.run(host="0.0.0.0", port=5000)