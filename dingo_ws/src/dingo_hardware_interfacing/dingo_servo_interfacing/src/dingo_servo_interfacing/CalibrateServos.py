#!/usr/bin/env python3
# coding: utf-8
import sys
import numpy as np
import math as m

from ServoCalibrationDefinition import motor_config

# FIRST define a new motor class
Dingo  = motor_config()

r'''    HOW TO CALIBRATE THE MOTORS
This is how the robot should look at the calibbration position of [0,0,90]
                            LINKAGE
                          /‾‾‾‾‾‾‾\------------------- q
                         /   _______                   |
                        |   |    o__|___UPPER LEG______/   <---- UPPER LEG AT 0° POINTS HORIZONTALLY BACKWARD
   LOWER LEG SERVO -->  |___|__o    |‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾/
      AT 90° POINTS         |       |                /
   HORIZONTALLY FORWARD      ‾‾‾‾‾‾‾                /
                            SERVO HUB              / LOWER LEG
                                                  /
                                                 O

CALIBRATION PROCESS
1. Mount servo hubs (without legs) on hip servos at approximately 90 degrees (middle of the servo's range)
2. Ensure all motors are powered and run this script with:
            pos = calibration_pos
                                                and,
            offsets = np.array(
                    [[88, 0, 115, 86],
                    [5, 0, 15, 7],
                    [0, 0, 35, 0]])
3. Mount upper leg and lower leg servo horn **such that a positive calibration angle will achieve the desired position**.
    So, upper leg should be slightly angled up toward the back of the robot and lower leg servo horn
    should be slightly angled down from the forward horizontal
4. Run this script repeatedly and adjust calibration offsets until the deesired position is reached.
    It is suggested to calibrate hips first.

    HIP   servos: positive angles rotate the hip up
    UPPER servos: positive angles rotate clockwise for left and anticlockwise for right (down on diagram)
    LOWER servos: positive angles rotate clockwise for left and anticlockwise for right (up on diagram)
5. Once calibration offsets have all been found, copy values of "offsets" array to the hardware interface
    and replace values of "self.physical_calibration_offsets"

'''

#-------- MOVING CALIBRATED LEGS TO THE HOME POSITION -------- #
# ## Home position values:
calibration_pos = [0,0,90] # [hip_servo angle, upper leg servo angle,lower leg servo angle]


# These three positions are presets for the robot standing in a low, medium and high stance.
# Used for testing only
low = [0,25,140]
mid = [0,42,120]
high = [0,50,110]

position_dict = {
    "cal": calibration_pos,
    "low": low,
    "mid": mid,
    "high": high
}

servo_dict = {
    "fr",
    "fl",
    "br",
    "bl",
    "all",
    "relax"
}

#  CHOOSE POSITION:
# pos = calibration_pos

# Web UI HTML template
HTML_CONTENT = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Dingo Quadruped Servo Calibration</title>
    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --text-color: #f8fafc;
            --text-muted: #94a3b8;
            --accent-color: #3b82f6;
            --accent-hover: #2563eb;
            --success-color: #10b981;
            --border-color: #334155;
            --highlight-color: #f59e0b;
        }
        body {
            font-family: 'Inter', system-ui, -apple-system, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            margin: 0;
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
            min-height: 100vh;
        }
        header {
            margin-bottom: 20px;
            text-align: center;
        }
        h1 {
            margin: 0 0 10px 0;
            font-size: 2rem;
            letter-spacing: -0.025em;
        }
        p {
            color: var(--text-muted);
            margin: 0;
        }
        .container {
            width: 100%;
            max-width: 1200px;
            display: grid;
            grid-template-columns: 1fr;
            gap: 20px;
        }
        @media(min-width: 768px) {
            .container {
                grid-template-columns: repeat(2, 1fr);
            }
        }
        .leg-card {
            background-color: var(--card-bg);
            border: 2px solid var(--border-color);
            border-radius: 12px;
            padding: 20px;
            transition: border-color 0.2s;
        }
        .leg-card.focused {
            border-color: var(--highlight-color);
        }
        .leg-title {
            font-size: 1.25rem;
            font-weight: 600;
            margin-bottom: 15px;
            border-bottom: 1px solid var(--border-color);
            padding-bottom: 8px;
        }
        .joint-row {
            display: flex;
            flex-direction: column;
            margin-bottom: 15px;
            padding: 8px;
            border-radius: 8px;
            cursor: pointer;
            transition: background-color 0.2s;
            border: 1px solid transparent;
        }
        .joint-row:hover {
            background-color: rgba(255, 255, 255, 0.03);
        }
        .joint-row.active {
            background-color: rgba(245, 158, 11, 0.1);
            border: 1px solid var(--highlight-color);
        }
        .joint-info {
            display: flex;
            justify-content: space-between;
            margin-bottom: 6px;
        }
        .joint-name {
            font-weight: 500;
        }
        .joint-values {
            font-family: monospace;
            font-size: 0.95rem;
        }
        .slider-container {
            display: flex;
            align-items: center;
            gap: 12px;
        }
        input[type="range"] {
            flex-grow: 1;
            accent-color: var(--accent-color);
            cursor: pointer;
        }
        .save-bar {
            width: 100%;
            max-width: 1200px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-top: 20px;
            padding: 15px 20px;
            background-color: var(--card-bg);
            border-radius: 12px;
            border: 1px solid var(--border-color);
        }
        button {
            background-color: var(--success-color);
            color: white;
            border: none;
            border-radius: 6px;
            padding: 12px 24px;
            font-size: 1rem;
            font-weight: 600;
            cursor: pointer;
            transition: opacity 0.2s;
        }
        button:hover {
            opacity: 0.9;
        }
        .status-msg {
            font-weight: 500;
        }
        .instructions {
            background-color: rgba(59, 130, 246, 0.1);
            border: 1px solid var(--accent-color);
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            font-size: 0.9rem;
            line-height: 1.5;
            max-width: 1200px;
        }
        .instructions code {
            background-color: rgba(255, 255, 255, 0.1);
            padding: 2px 4px;
            border-radius: 4px;
        }
        .preset-container {
            width: 100%;
            max-width: 1200px;
            display: flex;
            gap: 12px;
            align-items: center;
            margin-bottom: 20px;
            background-color: var(--card-bg);
            border-radius: 12px;
            border: 1px solid var(--border-color);
            padding: 15px 20px;
            box-sizing: border-box;
        }
        .preset-label {
            font-weight: 600;
            margin-right: 10px;
        }
        .preset-btn {
            background-color: var(--border-color);
            color: var(--text-color);
            border: 1px solid rgba(255,255,255,0.1);
            padding: 8px 16px;
            font-size: 0.9rem;
            font-weight: 500;
            border-radius: 6px;
            cursor: pointer;
            transition: all 0.2s;
        }
        .preset-btn:hover {
            background-color: var(--accent-color);
            border-color: var(--accent-color);
        }
        .preset-btn.active {
            background-color: var(--accent-color);
            border-color: var(--accent-color);
            box-shadow: 0 0 10px rgba(59, 130, 246, 0.5);
        }
    </style>
</head>
<body>
    <header>
        <h1>Dingo Quadruped Servo Calibration</h1>
        <p>Interactive calibration interface</p>
    </header>

    <div class="instructions">
        <strong>Controls:</strong> Click any joint to select it. Adjust the value using the <strong>sliders</strong> or the <strong>Left / Right Arrow keys</strong> (fine-tune by 1 degree). Move selection with <strong>Up / Down Arrow keys</strong>. Press <strong>Enter</strong> or click "Save Calibration" to write updates directly to the source code files.
    </div>

    <div class="preset-container">
        <span class="preset-label">Stance Presets:</span>
        <button class="preset-btn" onclick="setStance('cal')">Calibration (Cal)</button>
        <button class="preset-btn" onclick="setStance('low')">Low</button>
        <button class="preset-btn" onclick="setStance('mid')">Medium</button>
        <button class="preset-btn" onclick="setStance('high')">High</button>
        
        <span style="flex-grow: 1;"></span>
        
        <label style="display: flex; align-items: center; gap: 8px; font-weight: 500; cursor: pointer; user-select: none;">
            <input type="checkbox" id="soft-motion-checkbox" onchange="toggleSoftMotion(this.checked)" style="width: 18px; height: 18px; cursor: pointer;">
            Use Soft Motion
        </label>
    </div>

    <div class="container" id="legs-container">
        <!-- Generated Dynamically -->
    </div>

    <div class="save-bar">
        <span class="status-msg" id="status-text">Offsets loaded. Click "Save Calibration" to persist changes.</span>
        <button id="save-btn" onclick="saveCalibration()">Save Calibration</button>
    </div>

    <script>
        const legNames = ["Front-Right (FR)", "Front-Left (FL)", "Back-Right (BR)", "Back-Left (BL)"];
        const jointNames = ["Hip", "Upper Leg", "Lower Leg"];
        let offsets = [[0,0,0,0], [0,0,0,0], [0,0,0,0]];
        let pins = [[0,0,0,0], [0,0,0,0], [0,0,0,0]];
        let calibrationPos = [0, 0, 90];
        
        let selectedLeg = 0; // 0..3
        let selectedJoint = 0; // 0..2
        let activeStance = 'cal';

        async function fetchOffsets() {
            try {
                const response = await fetch('/get_offsets');
                const data = await response.json();
                offsets = data.offsets;
                calibrationPos = data.calibration_pos;
                pins = data.pins || pins;
                detectStance();
                renderUI();
                
                // Fetch soft motion state
                const smRes = await fetch('/get_soft_motion');
                const smData = await smRes.json();
                document.getElementById('soft-motion-checkbox').checked = smData.use_soft_motion;
            } catch (err) {
                console.error("Failed to load offsets", err);
                document.getElementById('status-text').innerText = "Error loading offsets.";
            }
        }

        function detectStance() {
            if (calibrationPos[0] === 0 && calibrationPos[1] === 25 && calibrationPos[2] === 140) activeStance = 'low';
            else if (calibrationPos[0] === 0 && calibrationPos[1] === 42 && calibrationPos[2] === 120) activeStance = 'mid';
            else if (calibrationPos[0] === 0 && calibrationPos[1] === 50 && calibrationPos[2] === 110) activeStance = 'high';
            else activeStance = 'cal';
            updateActiveStanceUI();
        }

        function updateActiveStanceUI() {
            document.querySelectorAll('.preset-btn').forEach(btn => {
                const isMatch = btn.getAttribute('onclick').includes(`'${activeStance}'`);
                if (isMatch) {
                    btn.classList.add('active');
                } else {
                    btn.classList.remove('active');
                }
            });
        }

        async function setStance(stance) {
            try {
                const response = await fetch('/set_stance', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ stance: stance })
                });
                const data = await response.json();
                if (data.status === "ok") {
                    calibrationPos = data.calibration_pos;
                    activeStance = stance;
                    updateActiveStanceUI();
                    renderUI();
                }
            } catch (err) {
                console.error("Failed to set stance", err);
            }
        }

        async function toggleSoftMotion(enabled) {
            try {
                await fetch('/set_soft_motion', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ use_soft_motion: enabled })
                });
            } catch (err) {
                console.error("Failed to toggle soft motion", err);
            }
        }

        function renderUI() {
            const container = document.getElementById('legs-container');
            container.innerHTML = "";
            
            for (let legIdx = 0; legIdx < 4; legIdx++) {
                const card = document.createElement('div');
                card.className = `leg-card ${legIdx === selectedLeg ? 'focused' : ''}`;
                
                const title = document.createElement('div');
                title.className = 'leg-title';
                title.innerText = legNames[legIdx];
                card.appendChild(title);
                
                for (let jointIdx = 0; jointIdx < 3; jointIdx++) {
                    const row = document.createElement('div');
                    const isActive = legIdx === selectedLeg && jointIdx === selectedJoint;
                    row.className = `joint-row ${isActive ? 'active' : ''}`;
                    row.onclick = () => {
                        selectedLeg = legIdx;
                        selectedJoint = jointIdx;
                        renderUI();
                    };
                    
                    const info = document.createElement('div');
                    info.className = 'joint-info';
                    
                    const name = document.createElement('span');
                    name.className = 'joint-name';
                    name.innerText = `${jointNames[jointIdx]} (Pin ${pins[jointIdx][legIdx]})`;
                    
                    const valDisplay = document.createElement('span');
                    valDisplay.className = 'joint-values';
                    const offsetVal = offsets[jointIdx][legIdx];
                    const totalVal = offsetVal + calibrationPos[jointIdx];
                    valDisplay.innerText = `Offset: ${offsetVal}° (Total: ${totalVal}°)`;
                    
                    info.appendChild(name);
                    info.appendChild(valDisplay);
                    row.appendChild(info);
                    
                    const sliderContainer = document.createElement('div');
                    sliderContainer.className = 'slider-container';
                    
                    const slider = document.createElement('input');
                    slider.type = 'range';
                    slider.min = '0';
                    slider.max = '180';
                    slider.value = offsetVal;
                    slider.oninput = (e) => {
                        updateOffset(legIdx, jointIdx, parseInt(e.target.value));
                    };
                    
                    sliderContainer.appendChild(slider);
                    row.appendChild(sliderContainer);
                    card.appendChild(row);
                }
                container.appendChild(card);
            }
        }

        async function updateOffset(legIdx, jointIdx, val) {
            offsets[jointIdx][legIdx] = val;
            
            // Re-render only labels to prevent slider stuttering
            const card = document.getElementById('legs-container').children[legIdx];
            const row = card.children[jointIdx + 1];
            const valDisplay = row.querySelector('.joint-values');
            const totalVal = val + calibrationPos[jointIdx];
            valDisplay.innerText = `Offset: ${val}° (Total: ${totalVal}°)`;
            
            try {
                const response = await fetch('/set_offset', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ leg_idx: legIdx, joint_idx: jointIdx, value: val })
                });
            } catch (err) {
                console.error("Failed to update servo position", err);
            }
        }

        async function saveCalibration() {
            const statusText = document.getElementById('status-text');
            statusText.innerText = "Saving offsets...";
            try {
                const response = await fetch('/save_offsets', { method: 'POST' });
                const data = await response.json();
                if (data.status === "saved") {
                    statusText.innerText = "Successfully saved calibration offsets to source files!";
                    statusText.style.color = "var(--success-color)";
                    setTimeout(() => {
                        statusText.innerText = "Offsets loaded. Click \\"Save Calibration\\" to persist changes.";
                        statusText.style.color = "";
                    }, 4000);
                } else {
                    statusText.innerText = "Save failed: " + data.status;
                }
            } catch (err) {
                statusText.innerText = "Failed to save calibration.";
                console.error(err);
            }
        }

        // Keyboard navigation
        window.addEventListener('keydown', (e) => {
            if (e.key === 'ArrowUp') {
                e.preventDefault();
                selectedJoint--;
                if (selectedJoint < 0) {
                    selectedJoint = 2;
                    selectedLeg = (selectedLeg - 1 + 4) % 4;
                }
                renderUI();
            } else if (e.key === 'ArrowDown') {
                e.preventDefault();
                selectedJoint++;
                if (selectedJoint > 2) {
                    selectedJoint = 0;
                    selectedLeg = (selectedLeg + 1) % 4;
                }
                renderUI();
            } else if (e.key === 'ArrowLeft') {
                e.preventDefault();
                let currentVal = offsets[selectedJoint][selectedLeg];
                if (currentVal > 0) {
                    updateOffset(selectedLeg, selectedJoint, currentVal - 1);
                    renderUI();
                }
            } else if (e.key === 'ArrowRight') {
                e.preventDefault();
                let currentVal = offsets[selectedJoint][selectedLeg];
                if (currentVal < 180) {
                    updateOffset(selectedLeg, selectedJoint, currentVal + 1);
                    renderUI();
                }
            } else if (e.key === 'Enter') {
                e.preventDefault();
                saveCalibration();
            }
        });

        // Initial Load
        fetchOffsets();
    </script>
</body>
</html>
"""

offsets = np.array(
                    [[88, 0, 115, 86],
                    [5, 0, 15, 7],
                    [0, 0, 35, 0]])

# Soft motion implementation
import threading
import time

target_angles = [None] * 16
current_angles = [None] * 16
use_soft_motion = False

def init_angles():
    global target_angles, current_angles
    for leg_idx in range(4):
        for joint_idx in range(3):
            val = offsets[joint_idx, leg_idx]
            target_angle = val + calibration_pos[joint_idx]
            
            if leg_idx == 0: # FR
                ch = [Dingo.front_right_hip, Dingo.front_right_upper, Dingo.front_right_lower][joint_idx]
            elif leg_idx == 1: # FL
                ch = [Dingo.front_left_hip, Dingo.front_left_upper, Dingo.front_left_lower][joint_idx]
            elif leg_idx == 2: # BR
                ch = [Dingo.back_right_hip, Dingo.back_right_upper, Dingo.back_right_lower][joint_idx]
            elif leg_idx == 3: # BL
                ch = [Dingo.back_left_hip, Dingo.back_left_upper, Dingo.back_left_lower][joint_idx]
            
            target_angles[ch] = float(target_angle)
            current_angles[ch] = float(target_angle)

def command_servo_angle(servo_pin, target_angle):
    global target_angles, current_angles, use_soft_motion
    target_angles[servo_pin] = float(target_angle)
    if current_angles[servo_pin] is None:
        current_angles[servo_pin] = float(target_angle)
        
    if not use_soft_motion:
        current_angles[servo_pin] = float(target_angle)
        try:
            Dingo.moveAbsAngle(servo_pin, target_angle)
        except Exception as e:
            print(f"Error moving servo on pin {servo_pin}: {e}")

def soft_motion_thread_func():
    global current_angles, target_angles, use_soft_motion
    while True:
        if use_soft_motion:
            for ch in range(16):
                if target_angles[ch] is not None and current_angles[ch] is not None:
                    error = target_angles[ch] - current_angles[ch]
                    if abs(error) > 0.05:
                        abs_err = abs(error)
                        if abs_err > 60:
                            step = 8.0
                        elif abs_err > 30:
                            step = 5.0
                        elif abs_err > 15:
                            step = 3.0
                        elif abs_err > 5:
                            step = 2.0
                        else:
                            step = 1.0
                            
                        if error > 0:
                            current_angles[ch] = min(target_angles[ch], current_angles[ch] + step)
                        else:
                            current_angles[ch] = max(target_angles[ch], current_angles[ch] - step)
                        
                        try:
                            Dingo.moveAbsAngle(ch, current_angles[ch])
                        except Exception as e:
                            pass
        time.sleep(0.02)

# Check if we should launch web server
if "--web" in sys.argv:
    import http.server
    import urllib.parse
    import json
    import re
    import webbrowser
    import os

    class CalibrationHTTPRequestHandler(http.server.BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            return

        def do_GET(self):
            parsed_url = urllib.parse.urlparse(self.path)
            path = parsed_url.path
            if path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(HTML_CONTENT.encode("utf-8"))
            elif path == "/get_offsets":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                data = {
                    "offsets": offsets.tolist(),
                    "calibration_pos": calibration_pos,
                    "pins": Dingo.pins.tolist()
                }
                self.wfile.write(json.dumps(data).encode("utf-8"))
            elif path == "/get_soft_motion":
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"use_soft_motion": use_soft_motion}).encode("utf-8"))
            else:
                self.send_error(404, "Not Found")

        def do_POST(self):
            global calibration_pos, offsets
            parsed_url = urllib.parse.urlparse(self.path)
            path = parsed_url.path
            
            if path == "/set_stance":
                content_length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_length)
                params = json.loads(body.decode('utf-8'))
                
                stance_name = params.get("stance")
                if stance_name in position_dict:
                    global calibration_pos
                    calibration_pos = position_dict[stance_name]
                    
                    # Move all servos to their new stance position
                    for leg_idx in range(4):
                        for joint_idx in range(3):
                            val = offsets[joint_idx, leg_idx]
                            target_angle = val + calibration_pos[joint_idx]
                            
                            if leg_idx == 0: # FR
                                servo_pin = [Dingo.front_right_hip, Dingo.front_right_upper, Dingo.front_right_lower][joint_idx]
                            elif leg_idx == 1: # FL
                                servo_pin = [Dingo.front_left_hip, Dingo.front_left_upper, Dingo.front_left_lower][joint_idx]
                            elif leg_idx == 2: # BR
                                servo_pin = [Dingo.back_right_hip, Dingo.back_right_upper, Dingo.back_right_lower][joint_idx]
                            elif leg_idx == 3: # BL
                                servo_pin = [Dingo.back_left_hip, Dingo.back_left_upper, Dingo.back_left_lower][joint_idx]
                            
                            try:
                                command_servo_angle(servo_pin, target_angle)
                            except Exception as e:
                                print(f"Error moving servo on pin {servo_pin}: {e}")
                                
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok", "calibration_pos": calibration_pos}).encode("utf-8"))
                else:
                    self.send_response(400)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "error", "message": "Invalid Stance"}).encode("utf-8"))
            elif path == "/set_soft_motion":
                content_length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_length)
                params = json.loads(body.decode('utf-8'))
                global use_soft_motion
                use_soft_motion = bool(params.get("use_soft_motion", False))
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"status": "ok", "use_soft_motion": use_soft_motion}).encode("utf-8"))
            elif path == "/set_offset":
                content_length = int(self.headers.get('Content-Length', 0))
                body = self.rfile.read(content_length)
                params = json.loads(body.decode('utf-8'))
                
                leg_idx = int(params["leg_idx"])
                joint_idx = int(params["joint_idx"])
                val = int(params["value"])
                
                offsets[joint_idx, leg_idx] = val
                
                target_angle = val + calibration_pos[joint_idx]
                
                try:
                    if leg_idx == 0: # FR
                        servo_pin = [Dingo.front_right_hip, Dingo.front_right_upper, Dingo.front_right_lower][joint_idx]
                    elif leg_idx == 1: # FL
                        servo_pin = [Dingo.front_left_hip, Dingo.front_left_upper, Dingo.front_left_lower][joint_idx]
                    elif leg_idx == 2: # BR
                        servo_pin = [Dingo.back_right_hip, Dingo.back_right_upper, Dingo.back_right_lower][joint_idx]
                    elif leg_idx == 3: # BL
                        servo_pin = [Dingo.back_left_hip, Dingo.back_left_upper, Dingo.back_left_lower][joint_idx]
                    
                    command_servo_angle(servo_pin, target_angle)
                    
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "ok", "target_angle": target_angle}).encode("utf-8"))
                except Exception as e:
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))
                    
            elif path == "/save_offsets":
                try:
                    file_path = os.path.abspath(__file__)
                    with open(file_path, "r", encoding="utf-8") as f:
                        content = f.read()
                    
                    new_offsets_str = f"offsets = np.array(\n                    [[{offsets[0,0]}, {offsets[0,1]}, {offsets[0,2]}, {offsets[0,3]}],\n                    [{offsets[1,0]}, {offsets[1,1]}, {offsets[1,2]}, {offsets[1,3]}],\n                    [{offsets[2,0]}, {offsets[2,1]}, {offsets[2,2]}, {offsets[2,3]}]])"
                    
                    pattern = r"offsets\s*=\s*np\.array\(\s*\[\[.*?\]\]\)"
                    content_new, count = re.subn(pattern, new_offsets_str, content, flags=re.DOTALL)
                    
                    if count > 0:
                        with open(file_path, "w", encoding="utf-8") as f:
                            f.write(content_new)
                        status = "saved"
                    else:
                        status = "regex_failed"
                    
                    # Also try to update HardwareInterface.py if present in the same folder
                    hw_path = os.path.join(os.path.dirname(file_path), "HardwareInterface.py")
                    if os.path.exists(hw_path):
                        with open(hw_path, "r", encoding="utf-8") as f:
                            hw_content = f.read()
                        new_hw_offsets_str = f"self.physical_calibration_offsets = np.array(\n                    [[{offsets[0,0]}, {offsets[0,1]}, {offsets[0,2]}, {offsets[0,3]}],\n                    [{offsets[1,0]}, {offsets[1,1]}, {offsets[1,2]}, {offsets[1,3]}],\n                    [{offsets[2,0]}, {offsets[2,1]}, {offsets[2,2]}, {offsets[2,3]}]])"
                        hw_pattern = r"self\.physical_calibration_offsets\s*=\s*np\.array\(\s*\[\[.*?\]\]\)"
                        hw_content_new, hw_count = re.subn(hw_pattern, new_hw_offsets_str, hw_content, flags=re.DOTALL)
                        if hw_count > 0:
                            with open(hw_path, "w", encoding="utf-8") as f:
                                f.write(hw_content_new)
                            status = "saved"
                    
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": status}).encode("utf-8"))
                except Exception as e:
                    self.send_response(500)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(json.dumps({"status": "error", "message": str(e)}).encode("utf-8"))

    init_angles()
    t = threading.Thread(target=soft_motion_thread_func, daemon=True)
    t.start()
    
    port = 8080
    server_address = ('', port)
    httpd = http.server.HTTPServer(server_address, CalibrationHTTPRequestHandler)
    print(f"Starting Dingo Calibration Web Server on port {port}...")
    print(f"Open your browser and navigate to http://localhost:{port}")
    webbrowser.open(f"http://localhost:{port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")
        sys.exit(0)

else:
    # Legacy CLI behavior
    servo_move = ""
    if len(sys.argv) > 2 and sys.argv[2] in position_dict:
        servo_move = sys.argv[2]
        pos = position_dict[servo_move]
    else:
        pos = calibration_pos

    servo_name = ""
    if len(sys.argv) > 1 and sys.argv[1] in servo_dict:
        servo_name = sys.argv[1]

        if servo_name != "relax":
            print('DINGO: Motor ' + servo_name + ' moved to ' + servo_move + '.\n')
        else:
            print('DINGO: Motors Relaxed.\n')

    if servo_name == "fr" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.front_right_hip  ,offsets[0,0]+pos[0])
        Dingo.moveAbsAngle(Dingo.front_right_upper,offsets[1,0]+pos[1])
        Dingo.moveAbsAngle(Dingo.front_right_lower,offsets[2,0]+pos[2])

    if servo_name == "fl" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.front_left_hip   ,offsets[0,1]+pos[0])
        Dingo.moveAbsAngle(Dingo.front_left_upper ,offsets[1,1]+pos[1])
        Dingo.moveAbsAngle(Dingo.front_left_lower ,offsets[2,1]+pos[2])

    if servo_name == "br" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.back_right_hip   ,offsets[0,2]+pos[0])
        Dingo.moveAbsAngle(Dingo.back_right_upper ,offsets[1,2]+pos[1])
        Dingo.moveAbsAngle(Dingo.back_right_lower ,offsets[2,2]+pos[2])

    if servo_name == "bl" or servo_name == "all":
        Dingo.moveAbsAngle(Dingo.back_left_hip    ,offsets[0,3]+pos[0])
        Dingo.moveAbsAngle(Dingo.back_left_upper  ,offsets[1,3]+pos[1])
        Dingo.moveAbsAngle(Dingo.back_left_lower  ,offsets[2,3]+pos[2])

    if servo_name == "relax":
        Dingo.relax_all_motors()



