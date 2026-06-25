#!/usr/bin/env python3
"""
RoboDog Quadruped - Real-Time 3D Interactive Calibration & Control GUI
Host PC Brain Architecture (Pure Microcontroller Output Driver model)

Features:
- Bootup safety: Microcontroller starts with completely limp servos (no boot snaps/jerks).
- Complete PC brain control: All offsets, inversions, and poses calculated on host PC.
- Direct raw physical angle writes (0.0-180.0) pushed live over serial.
- Matplotlib-based real-time 3D quadruped kinematic skeleton simulator.
- PC Storage (robodog_config.json) holds persistent calibration configurations.
"""

import sys
import os
import time
import math
import tkinter as tk
from tkinter import ttk, messagebox

# Try to import serial and matplotlib, otherwise show elegant install prompt
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Warning: 'pyserial' not installed. Install with: pip install pyserial")
    serial = None

try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
except ImportError:
    print("Warning: 'matplotlib' not installed. Install with: pip install matplotlib")
    matplotlib = None

# Joint Mapping Reference
JOINT_NAMES = [
    "FR Hip (Ch 14)", "FR Upper (Ch 13)", "FR Lower (Ch 12)",
    "FL Hip (Ch 10)", "FL Upper (Ch  9)", "FL Lower (Ch  8)",
    "BR Hip (Ch  2)", "BR Upper (Ch  1)", "BR Lower (Ch  0)",
    "BL Hip (Ch  6)", "BL Upper (Ch  5)", "BL Lower (Ch  4)"
]

# Physical dimensions for RoboDog 3D Kinematics (in meters)
BODY_LENGTH = 0.28
BODY_WIDTH = 0.15
L_HIP = 0.045
L_UPPER = 0.095
L_LOWER = 0.110


def calculate_4_bar(th2, a, b, c, d):
    # Freudenstein's method to solve a 4-bar linkage vertices ABCD
    try:
        x_b = a * math.cos(th2)
        y_b = a * math.sin(th2)
        
        f = math.sqrt((d - x_b)**2 + y_b**2)
        val = (f**2 + c**2 - b**2) / (2.0 * f * c)
        val = max(-1.0, min(1.0, val)) # Prevent DomainError
        beta = math.acos(val)
        gamma = math.atan2(y_b, d - x_b)
        
        th4 = math.pi - gamma - beta
        
        x_c = c * math.cos(th4) + d
        y_c = c * math.sin(th4)
        
        th3 = math.atan2((y_c - y_b), (x_c - x_b))
        
        ABC = math.pi - th2 + th3
        BCD = th4 - th3
        CDA = math.pi * 2.0 - th2 - ABC - BCD
        return ABC, BCD, CDA
    except Exception:
        return 0.0, 0.0, 0.0

def lower_leg_angle_to_servo_angle(theta_upper_rad, theta_lower_rad):
    # Linkage lengths in mm (from Config.py Leg_linkage)
    a = 35.12
    b = 37.6
    c = 43.0
    d = 35.23
    e = 67.1
    f = 130.0
    g = 37.0
    h = 43.0
    gamma = math.atan(28.80 / 20.20)
    try:
        EDC = math.acos((c**2 + h**2 - e**2) / (2.0 * c * h))
    except Exception:
        EDC = 1.57
    i = 130.0 # upper leg length L2 * 1000

    # First 4 bar linkage GDE, DEF, EFG
    GDE, DEF, EFG = calculate_4_bar(theta_lower_rad, i, h, f, g)
    
    # Triangle section CDH and CDA
    CDH = 1.5 * math.pi - theta_upper_rad - GDE - EDC
    CDA = CDH + gamma
    
    # Second 4 bar linkage DAB, ABC, BCD
    DAB, ABC, BCD = calculate_4_bar(CDA, d, a, b, c)
    
    # Calculating Theta
    THETA0 = DAB + gamma
    return THETA0

class RoboDogGuiApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RoboDog Quadruped 3D Controller & Calibration Utility")
        self.geometry("1280x880")
        self.configure(bg="#121214")

        # Serial state
        # Serial state
        self.ser = None
        self.offsets = [0] * 12
        self.angles = [90.0] * 12
        self.joint_entries = []
        
        # Default physical multipliers (can be toggled in GUI)
        self.inversions = [
            True, False, False,  # FR: Hip (inverted), Upper, Lower
            False, True, True,   # FL: Hip, Upper (inverted), Lower (inverted)
            False, False, False, # BR: Hip, Upper, Lower
            True, True, True     # BL: Hip (inverted), Upper (inverted), Lower (inverted)
        ]
        
        # Knee-Thigh parallel linkage coupling calibration factors (compensates for mechanical non-idealities)
        self.coupling_factors = [0.0] * 4

        # Teleoperation & Gait variables
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0
        self.body_height = -0.205  # m (neutral position is -20.5 cm)
        self.ticks = 0
        self.gait_active = False
        self.gait_ticks_remaining = 0
        self.current_height_level = 0

        # Style configurations
        self.setup_styles()
        
        # Build UI layout
        self.create_widgets()
        
        # Initialize 3D Plot
        if matplotlib:
            self.init_3d_plot()
            self.update_3d_view()
        else:
            self.show_matplotlib_warning()

        # Search serial ports
        self.scan_ports()

        # Load existing calibration config from PC on startup
        self.load_from_pc(auto_push=False)

        # Initialize Body Height to Neutral
        self.set_height_level(0)

        # Track pending Tkinter after callbacks for graceful shutdown
        self.dither_after_id = None
        self.gait_after_id = None
        
        # Intercept window close event to shut down gracefully
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # Start background dither/micro-oscillation loop for servo position retention
        self.start_dither_loop()

    def start_dither_loop(self):
        self.last_target_angles = [90.0] * 12
        self.last_target_change_time = [time.time()] * 12
        self.dither_index = 0
        self.dither_sequence = [1.0, 0.0]
        self.dither_loop()

    def dither_loop(self):
        # Schedule the next run in 200 ms (5 times a second)
        self.dither_after_id = self.after(200, self.dither_loop)
        
        if not self.ser or not self.ser.is_open:
            return
            
        current_time = time.time()
        
        # 1. Update last change times based on self.angles changes
        for i in range(12):
            if abs(self.angles[i] - self.last_target_angles[i]) > 0.01:
                self.last_target_angles[i] = self.angles[i]
                self.last_target_change_time[i] = current_time
                
        # 2. Progress the dither pattern index
        self.dither_index = (self.dither_index + 1) % len(self.dither_sequence)
        current_offset = self.dither_sequence[self.dither_index]
        
        # 3. Calculate raw angles
        raw_angles = self.get_raw_servo_angles()
        dithered_angles = []
        any_stable = False
        
        for i in range(12):
            # A motor is stable if gait is not active AND its target angle hasn't changed for >= 0.5s
            is_stable = (not self.gait_active) and (current_time - self.last_target_change_time[i] >= 0.5)
            
            if is_stable:
                # Apply current step in the [1.0, 0.0] sequence
                dithered = raw_angles[i] + current_offset
                dithered = max(0.0, min(180.0, dithered))
                dithered_angles.append(dithered)
                any_stable = True
            else:
                dithered_angles.append(raw_angles[i])
                
        # 4. If any motors are stable, send the updated command with micro-oscillations to the Arduino
        if any_stable:
            self.push_all_angles(dithered_angles)

    def setup_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        
        # Configure dark mode theme with large, clean typography
        style.configure(".", background="#121214", foreground="#e4e4e7", font=("Consolas", 12))
        style.configure("TLabel", background="#121214", foreground="#e4e4e7", font=("Consolas", 12))
        style.configure("TFrame", background="#121214")
        
        # Labelframe dark styling with prominent headers
        style.configure("TLabelframe", background="#1a1a1e", bordercolor="#2d2d34", relief="solid", borderwidth=1)
        style.configure("TLabelframe.Label", background="#1a1a1e", foreground="#38bdf8", font=("Consolas", 13, "bold"))
        
        # Custom button styles with padding and large fonts
        style.configure("TButton", background="#2d2d34", foreground="#e4e4e7", borderwidth=0, padding=8, relief="flat", font=("Consolas", 12, "bold"))
        style.map("TButton",
                  background=[("active", "#38bdf8"), ("pressed", "#0284c7")],
                  foreground=[("active", "#121214"), ("pressed", "#ffffff")])
                  
        style.configure("Action.TButton", background="#0284c7", foreground="#ffffff", font=("Consolas", 12, "bold"))
        style.configure("Alert.TButton", background="#ef4444", foreground="#ffffff", font=("Consolas", 12, "bold"))

        # Explicitly configure Combobox widget elements for perfect high contrast
        style.configure("TCombobox", 
                        fieldbackground="#1a1a1e", 
                        background="#2d2d34", 
                        foreground="#e4e4e7", 
                        arrowcolor="#e4e4e7",
                        bordercolor="#2d2d34",
                        font=("Consolas", 12))
        
        style.map("TCombobox", 
                  fieldbackground=[("readonly", "#1a1a1e")], 
                  foreground=[("readonly", "#e4e4e7")])

        # Style Entry fields
        style.configure("TEntry", fieldbackground="#1a1a1e", foreground="#e4e4e7", insertcolor="#e4e4e7", font=("Consolas", 12))

        # Style the dropdown selection listbox
        self.option_add("*TCombobox*Listbox.background", "#1a1a1e")
        self.option_add("*TCombobox*Listbox.foreground", "#e4e4e7")
        self.option_add("*TCombobox*Listbox.selectBackground", "#38bdf8")
        self.option_add("*TCombobox*Listbox.selectForeground", "#121214")
        self.option_add("*TCombobox*Listbox.font", ("Consolas", 12))

    def create_widgets(self):
        # Grid weights
        self.grid_columnconfigure(0, weight=2) # Left Control panel
        self.grid_columnconfigure(1, weight=3) # Right 3D Visualizer panel
        self.grid_rowconfigure(0, weight=1)

        # Left Column Frame (Control Panel)
        left_panel = ttk.Frame(self)
        left_panel.grid(row=0, column=0, sticky="nsew", padx=15, pady=15)
        left_panel.grid_columnconfigure(0, weight=1)
        left_panel.grid_rowconfigure(1, weight=1) # Let the joint list frame expand

        # 1. Connection Panel
        conn_frame = ttk.LabelFrame(left_panel, text=" OVER-THE-WIRE CONNECTION ")
        conn_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.port_combobox = ttk.Combobox(conn_frame, width=15)
        self.port_combobox.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection, style="Action.TButton")
        self.btn_connect.grid(row=0, column=2, padx=10, pady=5)
        
        self.btn_scan = ttk.Button(conn_frame, text="Scan Ports", command=self.scan_ports)
        self.btn_scan.grid(row=0, column=3, padx=5, pady=5)

        # 2. Joint Grid Panel
        self.joint_frame = ttk.LabelFrame(left_panel, text=" JOINT CALIBRATION & INVERSION GRID ")
        self.joint_frame.grid(row=1, column=0, sticky="nsew", pady=10)
        self.joint_frame.grid_columnconfigure(1, weight=1) # Scale slider

        # Header columns labels
        ttk.Label(self.joint_frame, text="Joint Description", font=("Consolas", 12, "bold")).grid(row=0, column=0, padx=5, pady=2, sticky="w")
        ttk.Label(self.joint_frame, text="Actuation Angle", font=("Consolas", 12, "bold")).grid(row=0, column=1, padx=5, pady=2)
        ttk.Label(self.joint_frame, text="Offset (Deg)", font=("Consolas", 12, "bold")).grid(row=0, column=2, padx=5, pady=2)
        ttk.Label(self.joint_frame, text="Invert", font=("Consolas", 12, "bold")).grid(row=0, column=3, padx=5, pady=2)

        # Build dynamic widgets for 12 joints
        self.joint_sliders = []
        self.offset_labels = []
        self.invert_vars = []
        
        for i in range(12):
            r = i + 1
            ttk.Label(self.joint_frame, text=JOINT_NAMES[i], width=20).grid(row=r, column=0, padx=5, pady=3, sticky="w")
            
            act_frame = ttk.Frame(self.joint_frame)
            act_frame.grid(row=r, column=1, padx=5, pady=3, sticky="ew")
            
            if i % 3 == 0:     # Hip
                s_from, s_to = 0, 180
            elif i % 3 == 1:   # Thigh/Upper
                s_from, s_to = 0, 160
            else:              # Calf/Lower (+80 to -80 software angle, corresponding to 10 to 170 slider angle)
                s_from, s_to = 10, 170
                
            slider = ttk.Scale(act_frame, from_=s_from, to=s_to, orient="horizontal", command=lambda val, idx=i: self.on_slider_move(idx, val))
            slider.set(90.0)
            slider.pack(side="left", fill="x", expand=True, padx=(0, 5))
            self.joint_sliders.append(slider)
            
            entry = ttk.Entry(act_frame, width=4, justify="center")
            entry.insert(0, "90")
            entry.pack(side="right")
            entry.bind("<KeyRelease>", lambda event, idx=i: self.on_entry_change(idx, event))
            self.joint_entries.append(entry)
            
            offset_subframe = ttk.Frame(self.joint_frame)
            offset_subframe.grid(row=r, column=2, padx=5, pady=3)
            
            btn_dec = ttk.Button(offset_subframe, text="-", width=2, command=lambda idx=i: self.adjust_offset(idx, -1))
            btn_dec.pack(side="left", padx=1)
            
            lbl_offset = ttk.Label(offset_subframe, text="0", width=4, anchor="center")
            lbl_offset.pack(side="left", padx=2)
            self.offset_labels.append(lbl_offset)
            
            btn_inc = ttk.Button(offset_subframe, text="+", width=2, command=lambda idx=i: self.adjust_offset(idx, 1))
            btn_inc.pack(side="left", padx=1)
            
            inv_var = tk.BooleanVar(value=self.inversions[i])
            chk_inv = ttk.Checkbutton(self.joint_frame, variable=inv_var, command=lambda idx=i: self.on_inversion_toggle(idx))
            chk_inv.grid(row=r, column=3, padx=5, pady=3)
            self.invert_vars.append(inv_var)

        # 3. Control Operations & Presets
        ops_frame = ttk.LabelFrame(left_panel, text=" HARDWARE POSES & CALIBRATION COMMS ")
        ops_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        
        poses_title = ttk.Label(ops_frame, text="Predefined Poses / Control Options:", font=("Consolas", 12, "bold"))
        poses_title.grid(row=0, column=0, columnspan=2, padx=5, pady=3, sticky="w")
        
        self.lock_knee = tk.BooleanVar(value=False)
        self.chk_lock_knee = ttk.Checkbutton(ops_frame, text="Lock Knee Joints", variable=self.lock_knee, command=self.on_lock_knee_toggle)
        self.chk_lock_knee.grid(row=0, column=2, columnspan=2, padx=5, pady=3, sticky="e")
        
        self.btn_stand = ttk.Button(ops_frame, text="Stand", command=lambda: self.trigger_pose("Stand"))
        self.btn_stand.grid(row=1, column=0, padx=5, pady=5, sticky="ew")
        
        self.btn_sit = ttk.Button(ops_frame, text="Sit", command=lambda: self.trigger_pose("Sit"))
        self.btn_sit.grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        self.btn_lay = ttk.Button(ops_frame, text="Lay Down", command=lambda: self.trigger_pose("Lay Down"))
        self.btn_lay.grid(row=1, column=2, padx=5, pady=5, sticky="ew")
        
        self.btn_neutral = ttk.Button(ops_frame, text="Neutral (90°)", command=lambda: self.trigger_pose("Neutral"))
        self.btn_neutral.grid(row=1, column=3, padx=5, pady=5, sticky="ew")

        # Relax All Servos Button
        self.btn_relax = ttk.Button(ops_frame, text="RELAX ALL SERVOS (LIMP)", style="Alert.TButton", command=self.relax_servos)
        self.btn_relax.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Limb Assembly Button
        self.btn_assembly = ttk.Button(ops_frame, text="Limb Assembly Mode (Raw)", style="Action.TButton", command=lambda: self.trigger_pose("Assembly"))
        self.btn_assembly.grid(row=2, column=2, columnspan=2, padx=5, pady=5, sticky="ew")

        # PC Storage Controls
        pc_title = ttk.Label(ops_frame, text="PC Storage Controls (robodog_config.json):", font=("Consolas", 12, "bold"))
        pc_title.grid(row=3, column=0, columnspan=4, padx=5, pady=(8, 3), sticky="w")
        
        self.btn_save = ttk.Button(ops_frame, text="SAVE CALIBRATION TO PC", style="Action.TButton", command=self.save_to_pc)
        self.btn_save.grid(row=4, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        self.btn_load = ttk.Button(ops_frame, text="RELOAD FROM PC", command=lambda: self.load_from_pc(auto_push=False))
        self.btn_load.grid(row=4, column=2, columnspan=2, padx=5, pady=5, sticky="ew")

        # Coupling calibration controls
        coupling_title = ttk.Label(ops_frame, text="Parallel Linkage Coupling Factor Compensation:", font=("Consolas", 11, "bold"))
        coupling_title.grid(row=5, column=0, columnspan=4, padx=5, pady=(8, 2), sticky="w")
        
        self.coupling_vars = []
        self.coupling_spinboxes = []
        
        coupling_frame = ttk.Frame(ops_frame)
        coupling_frame.grid(row=6, column=0, columnspan=4, sticky="ew", padx=5, pady=(0, 5))
        
        leg_prefixes = ["FR", "FL", "BR", "BL"]
        colors = ["#10b981", "#a855f7", "#f59e0b", "#ec4899"]
        
        for idx, (prefix, color) in enumerate(zip(leg_prefixes, colors)):
            coupling_frame.columnconfigure(idx, weight=1)
            sub = ttk.Frame(coupling_frame)
            sub.grid(row=0, column=idx, padx=2, pady=2, sticky="ew")
            
            ttk.Label(sub, text=f"{prefix}:", font=("Consolas", 10, "bold"), foreground=color).pack(side="left", padx=2)
            
            var = tk.DoubleVar(value=0.0)
            self.coupling_vars.append(var)
            
            sb = ttk.Spinbox(sub, textvariable=var, from_=-1.0, to=1.0, increment=0.05, width=5, 
                             command=lambda idx=idx: self.on_coupling_change(idx))
            sb.pack(side="left", padx=2)
            sb.bind("<KeyRelease>", lambda event, idx=idx: self.on_coupling_change(idx))
            self.coupling_spinboxes.append(sb)

        # 4. Flight Control Panel (Replacing Joysticks with Direct Step-wise control)
        self.teleop_frame = ttk.LabelFrame(left_panel, text=" DIRECT STEP-WISE FLIGHT CONTROL ")
        self.teleop_frame.grid(row=3, column=0, sticky="ew", pady=(10, 0))
        self.teleop_frame.grid_columnconfigure(0, weight=1)
        
        # 4.1 Height Level Controls Subframe
        height_panel = ttk.Frame(self.teleop_frame)
        height_panel.grid(row=0, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Label(height_panel, text="Body Height Adjustment:", font=("Consolas", 11, "bold")).pack(anchor="w", pady=(0, 2))
        
        btn_height_frame = ttk.Frame(height_panel)
        btn_height_frame.pack(fill="x", pady=2)
        
        # We want to create 7 buttons for levels: -3, -2, -1, 0 (Neutral), +1, +2, +3
        self.height_buttons = {}
        levels = [-3, -2, -1, 0, 1, 2, 3]
        labels = ["Drop -3", "Drop -2", "Drop -1", "Neutral", "Raise +1", "Raise +2", "Raise +3"]
        
        for lvl, lbl in zip(levels, labels):
            btn = ttk.Button(btn_height_frame, text=lbl, width=9, command=lambda l=lvl: self.set_height_level(l))
            btn.pack(side="left", padx=1, expand=True, fill="x")
            self.height_buttons[lvl] = btn
            
        self.lbl_height_status = ttk.Label(height_panel, text="Current Height: Neutral (-20.5 cm)", font=("Consolas", 10), foreground="#38bdf8")
        self.lbl_height_status.pack(anchor="w", pady=(2, 5))

        # 4.2 Locomotion Steps Control Subframe
        move_panel = ttk.Frame(self.teleop_frame)
        move_panel.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        
        ttk.Label(move_panel, text="Step-wise Locomotion Controls:", font=("Consolas", 11, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", pady=(5, 5))
        
        # Grid of movement buttons
        btn_walk_f1 = ttk.Button(move_panel, text="Walk Forward\n(1 Step)", width=18, command=lambda: self.execute_gait_steps(vx=0.12, vy=0.0, yaw_rate=0.0, ticks=16))
        btn_walk_f1.grid(row=1, column=0, padx=2, pady=2, sticky="ew")
        
        btn_walk_f5 = ttk.Button(move_panel, text="Walk Forward\n(5 Steps)", width=18, command=lambda: self.execute_gait_steps(vx=0.12, vy=0.0, yaw_rate=0.0, ticks=80))
        btn_walk_f5.grid(row=1, column=1, padx=2, pady=2, sticky="ew")
        
        btn_turn_l1 = ttk.Button(move_panel, text="Turn Left\n(1 Step)", width=18, command=lambda: self.execute_gait_steps(vx=0.0, vy=0.0, yaw_rate=-0.8, ticks=16))
        btn_turn_l1.grid(row=2, column=0, padx=2, pady=2, sticky="ew")
        
        btn_turn_r1 = ttk.Button(move_panel, text="Turn Right\n(1 Step)", width=18, command=lambda: self.execute_gait_steps(vx=0.0, vy=0.0, yaw_rate=0.8, ticks=16))
        btn_turn_r1.grid(row=2, column=1, padx=2, pady=2, sticky="ew")
        
        btn_turn_r5 = ttk.Button(move_panel, text="Turn Right\n(5 Steps)", width=18, command=lambda: self.execute_gait_steps(vx=0.0, vy=0.0, yaw_rate=0.8, ticks=80))
        btn_turn_r5.grid(row=2, column=2, padx=2, pady=2, sticky="ew")
        
        btn_walk_b1 = ttk.Button(move_panel, text="Walk Backward\n(1 Step)", width=18, command=lambda: self.execute_gait_steps(vx=-0.12, vy=0.0, yaw_rate=0.0, ticks=16))
        btn_walk_b1.grid(row=3, column=0, padx=2, pady=2, sticky="ew")
        
        btn_estop = ttk.Button(move_panel, text="E-STOP / HALT\n(Stance)", width=18, style="Alert.TButton", command=self.halt_gait)
        btn_estop.grid(row=3, column=1, columnspan=2, padx=2, pady=2, sticky="ew")
        
        # Configure columns of move_panel to expand equally
        for c in range(3):
            move_panel.grid_columnconfigure(c, weight=1)

        # Right Column Frame (3D Live Plot Panel + Live Kinematics Monitor)
        right_panel = ttk.Frame(self)
        right_panel.grid(row=0, column=1, sticky="nsew", padx=15, pady=15)
        right_panel.grid_columnconfigure(0, weight=1)
        right_panel.grid_rowconfigure(0, weight=3) # 3D Plot takes more weight
        right_panel.grid_rowconfigure(1, weight=2) # Monitor panel takes some weight
        
        self.plot_frame = ttk.LabelFrame(right_panel, text=" ROBODOG REAL-TIME KINEMATICS (3D WIREFRAME MODEL) ")
        self.plot_frame.grid(row=0, column=0, sticky="nsew")
        self.plot_frame.grid_columnconfigure(0, weight=1)
        self.plot_frame.grid_rowconfigure(0, weight=1)

        self.monitor_frame = ttk.LabelFrame(right_panel, text=" REAL-TIME KINEMATICS & SERVO MONITOR ")
        self.monitor_frame.grid(row=1, column=0, sticky="nsew", pady=(15, 0))
        for col in range(4):
            self.monitor_frame.grid_columnconfigure(col, weight=1)
            
        self.leg_monitor_labels = {}
        leg_names = ["FR Leg", "FL Leg", "BR Leg", "BL Leg"]
        colors = ["#10b981", "#a855f7", "#f59e0b", "#ec4899"] # Green, Purple, Yellow, Pink
        
        for idx, (name, color) in enumerate(zip(leg_names, colors)):
            frame = ttk.Frame(self.monitor_frame, padding=5)
            frame.grid(row=0, column=idx, sticky="nsew", padx=5, pady=5)
            
            title_lbl = ttk.Label(frame, text=name, font=("Consolas", 12, "bold"), foreground=color)
            title_lbl.pack(anchor="w", pady=(0, 2))
            
            pos_lbl = ttk.Label(frame, text="Pos (cm):\n  X: ---\n  Y: ---\n  Z: ---", font=("Consolas", 10), justify="left")
            pos_lbl.pack(anchor="w", pady=1)
            
            sw_lbl = ttk.Label(frame, text="SW Angles:\n  Hip: ---\n  Thigh: ---\n  Calf: ---", font=("Consolas", 10), justify="left")
            sw_lbl.pack(anchor="w", pady=1)
            
            raw_lbl = ttk.Label(frame, text="Servo Outputs:\n  Hip: ---\n  Thigh: ---\n  Calf: ---", font=("Consolas", 10), justify="left")
            raw_lbl.pack(anchor="w", pady=1)
            
            self.leg_monitor_labels[idx] = {
                "pos": pos_lbl,
                "sw": sw_lbl,
                "raw": raw_lbl
            }

    def scan_ports(self):
        if not serial:
            self.port_combobox["values"] = ["Mock Serial Port"]
            self.port_combobox.current(0)
            return
            
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        
        if port_list:
            self.port_combobox["values"] = port_list
            self.port_combobox.current(0)
        else:
            self.port_combobox["values"] = ["No ports found"]
            self.port_combobox.current(0)

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            try:
                # Set servos to neutral and disconnect
                neutral_angles = [90.0] * 12
                self.push_all_angles(neutral_angles)
                time.sleep(0.1)
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.btn_connect.configure(text="Connect", style="Action.TButton")
            print("Serial port disconnected.")
        else:
            # Connect
            port = self.port_combobox.get()
            if port == "No ports found" or port == "Mock Serial Port":
                messagebox.showwarning("Connection Failure", "Please select a valid hardware COM port.")
                return
                
            try:
                self.ser = serial.Serial(port, 115200, timeout=2.0)
                # Wait for Uno bootup sequence to complete fully
                time.sleep(3.0)
                self.ser.reset_input_buffer()
                
                # Push PC calibration settings to the board on connection
                self.load_from_pc(auto_push=True)
                
                self.btn_connect.configure(text="Disconnect", style="Alert.TButton")
                print(f"Connected to Arduino Uno on {port} and pushed PC calibration profile.")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to open serial connection:\n{e}")

    def sync_offset_labels(self):
        for i in range(12):
            self.offset_labels[i].configure(text=str(self.offsets[i]))

    def send_arduino_cmd_async(self, cmd_str):
        if not self.ser or not self.ser.is_open:
            return
        try:
            full_cmd = f"{cmd_str}\n"
            self.ser.write(full_cmd.encode('ascii'))
            self.ser.flush()
        except Exception as e:
            print(f"Serial communication error: {e}")

    def push_all_angles(self, raw_angles):
        JOINT_TO_CHANNEL = [14, 13, 12, 10, 9, 8, 2, 1, 0, 6, 5, 4]
        servo_angles = [90] * 16
        for i in range(12):
            channel = JOINT_TO_CHANNEL[i]
            servo_angles[channel] = int(round(raw_angles[i]))
        packet = ",".join(map(str, servo_angles))
        self.send_arduino_cmd_async(packet)

    def get_raw_servo_angles(self):
        raw_angles = [90.0] * 12
        legs_mapping = [
            (0, 1, 2, 0), # FR (h, u, l, leg_idx)
            (3, 4, 5, 1), # FL
            (6, 7, 8, 2), # BR
            (9, 10, 11, 3) # BL
        ]
        
        for h_idx, u_idx, l_idx, leg_idx in legs_mapping:
            hip_ang = self.angles[h_idx]
            up_ang = self.angles[u_idx]
            low_ang = self.angles[l_idx]
            
            # Hip raw
            offset_h = self.offsets[h_idx]
            inv_h = self.invert_vars[h_idx].get()
            r_h = (180.0 - (hip_ang + offset_h)) if inv_h else (hip_ang + offset_h)
            raw_angles[h_idx] = max(0.0, min(180.0, r_h))
            
            # Upper/Thigh raw
            offset_u = self.offsets[u_idx]
            inv_u = self.invert_vars[u_idx].get()
            r_u = (180.0 - (up_ang + offset_u)) if inv_u else (up_ang + offset_u)
            raw_angles[u_idx] = max(0.0, min(180.0, r_u))
            
            # Lower/Calf raw with linkage compensation AND physical linkage coupling correction!
            offset_l = self.offsets[l_idx]
            inv_l = self.invert_vars[l_idx].get()
            
            # If lock_knee is active, we use parallel linkage compensation.
            # If not active (direct calibration mode), we bypass compensation so the calf slider maps directly and independently to the servo!
            if hasattr(self, 'lock_knee') and self.lock_knee.get():
                THETA2 = math.radians(up_ang)
                drift_comp_factor = up_ang
                THETA3 = math.radians(low_ang - 90.0)
                
                THETA0 = lower_leg_angle_to_servo_angle(math.pi/2.0 - THETA2, THETA3 + math.pi/2.0)
                compensated_l = math.degrees(math.pi/2.0 + math.pi - THETA0)
                
                # Mechanical linkage non-ideality correction:
                # Compensate lower leg drift caused by thigh rotation
                coupling_factor = self.coupling_factors[leg_idx]
                drift_compensation = coupling_factor * drift_comp_factor
                compensated_l += drift_compensation
            else:
                compensated_l = low_ang
                
            r_l = (180.0 - (compensated_l + offset_l)) if inv_l else (compensated_l + offset_l)
            # Clamp raw servo output within standard physical limits [0, 180]
            r_l = max(0.0, min(180.0, r_l))
            raw_angles[l_idx] = r_l
            
        return raw_angles

    def on_coupling_change(self, leg_idx):
        try:
            val = float(self.coupling_vars[leg_idx].get())
            self.coupling_factors[leg_idx] = val
        except Exception:
            pass
        # Re-trigger Thigh slider update to instantly apply and push compensated calf angle!
        thigh_joints = [1, 4, 7, 10]
        self.on_slider_move(thigh_joints[leg_idx], self.joint_sliders[thigh_joints[leg_idx]].get())

    def solve_leg_inverse_kinematics(self, side, sx, sy, fx, fy, fz):
        # Translate foot target relative to shoulder
        x_rel = fx - sx
        y_rel = fy - sy
        z_rel = fz
        
        # 1. Solve Hip Angle
        d_yz = math.sqrt(y_rel**2 + z_rel**2)
        d_yz = max(L_HIP + 1e-4, d_yz)
        
        alpha = math.atan2(z_rel, y_rel * side)
        cos_val_hip = L_HIP / d_yz
        cos_val_hip = max(-1.0, min(1.0, cos_val_hip))
        beta = math.acos(cos_val_hip)
        theta_hip = alpha + beta
        
        # 2. Project to XZ_rotated plane
        x_rot = x_rel
        z_rot = -math.sqrt(max(0.0, d_yz**2 - L_HIP**2))
        
        # 3. Solve 2D Thigh and Calf angles
        d_pitch = math.sqrt(x_rot**2 + z_rot**2)
        d_pitch = min(L_UPPER + L_LOWER - 1e-4, max(0.05, d_pitch))
        
        # Thigh angle
        phi_1 = math.atan2(x_rot, -z_rot)
        cos_val_thigh = (L_UPPER**2 + d_pitch**2 - L_LOWER**2) / (2.0 * L_UPPER * d_pitch)
        cos_val_thigh = max(-1.0, min(1.0, cos_val_thigh))
        phi_2 = math.acos(cos_val_thigh)
        theta_upper = phi_1 - phi_2
        
        # Calf angle
        cos_knee = (L_UPPER**2 + L_LOWER**2 - d_pitch**2) / (2.0 * L_UPPER * L_LOWER)
        cos_knee = max(-1.0, min(1.0, cos_knee))
        theta_lower = math.acos(cos_knee) - math.pi
        
        # Convert back to slider angles (0 to 180, neutral is 90)
        hip_deg = math.degrees(theta_hip) + 90.0
        upper_deg = math.degrees(theta_upper) + 90.0
        lower_deg = math.degrees(theta_lower) + 90.0
        
        return hip_deg, upper_deg, lower_deg

    def set_height_level(self, level):
        level = max(-3, min(3, level))
        self.current_height_level = level
        self.body_height = -0.205 + level * 0.01
        
        lbl_text = "Current Height: "
        if level == 0:
            lbl_text += "Neutral (-20.5 cm)"
        elif level > 0:
            lbl_text += f"Raise +{level} ({self.body_height*100.0:.1f} cm)"
        else:
            lbl_text += f"Drop {level} ({self.body_height*100.0:.1f} cm)"
        self.lbl_height_status.configure(text=lbl_text)
        
        legs_config = [
            (-1, BODY_LENGTH/2, -BODY_WIDTH/2, 0, 1, 2, 0), # FR
            (1, BODY_LENGTH/2, BODY_WIDTH/2, 3, 4, 5, 1),  # FL
            (-1, -BODY_LENGTH/2, -BODY_WIDTH/2, 6, 7, 8, 2), # BR
            (1, -BODY_LENGTH/2, BODY_WIDTH/2, 9, 10, 11, 3) # BL
        ]
        
        for side, sx, sy, h_idx, u_idx, l_idx, leg_idx in legs_config:
            x0 = sx
            y0 = sy + side * L_HIP
            target_z = self.body_height
            
            hip_deg, upper_deg, lower_deg = self.solve_leg_inverse_kinematics(side, sx, sy, x0, y0, target_z)
            
            self.joint_sliders[h_idx].set(hip_deg)
            self.joint_sliders[u_idx].set(upper_deg - 90.0)
            self.joint_sliders[l_idx].set(lower_deg)
            self.angles[h_idx] = hip_deg
            self.angles[u_idx] = upper_deg - 90.0
            self.angles[l_idx] = lower_deg
            
        if self.ser and self.ser.is_open:
            raw_angles = self.get_raw_servo_angles()
            self.push_all_angles(raw_angles)
            
        self.update_3d_view()

    def execute_gait_steps(self, vx, vy, yaw_rate, ticks):
        self.vx = vx
        self.vy = vy
        self.yaw_rate = yaw_rate
        self.gait_ticks_remaining = ticks
        
        if not self.gait_active:
            self.gait_active = True
            self.ticks = 0
            self.gait_step_loop()
            print(f"Executing step-wise gait: vx={vx:.2f}, vy={vy:.2f}, yaw={yaw_rate:.2f} for {ticks} ticks.")

    def halt_gait(self):
        self.gait_active = False
        self.gait_ticks_remaining = 0
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0
        self.set_height_level(self.current_height_level)
        print("Gait execution halted. Reset to stance posture.")

    def gait_step_loop(self):
        if not self.gait_active:
            return
            
        if self.gait_ticks_remaining > 0:
            self.ticks += 1
            self.gait_ticks_remaining -= 1
            is_moving = True
        else:
            self.ticks = 0
            is_moving = False
            self.gait_active = False
            self.vx = 0.0
            self.vy = 0.0
            self.yaw_rate = 0.0
            
        # Trot parameters
        gait_cycle_ticks = 16.0
        half_cycle = gait_cycle_ticks / 2.0
        phase = self.ticks % int(gait_cycle_ticks)
        
        stride_scale = 0.35 # seconds
        z_clearance = 0.05 # 5 cm lift
        z0 = self.body_height
        
        legs_config = [
            (-1, BODY_LENGTH/2, -BODY_WIDTH/2, 0, 1, 2, 0), # FR
            (1, BODY_LENGTH/2, BODY_WIDTH/2, 3, 4, 5, 1),  # FL
            (-1, -BODY_LENGTH/2, -BODY_WIDTH/2, 6, 7, 8, 2), # BR
            (1, -BODY_LENGTH/2, BODY_WIDTH/2, 9, 10, 11, 3) # BL
        ]
        
        for side, sx, sy, h_idx, u_idx, l_idx, leg_idx in legs_config:
            is_pair_a = (leg_idx == 0 or leg_idx == 3)
            leg_phase = phase if is_pair_a else (phase + half_cycle) % gait_cycle_ticks
            
            x0 = sx
            y0 = sy + side * L_HIP
            
            if is_moving and leg_phase < half_cycle:
                # SWING
                swing_prop = leg_phase / half_cycle
                z_lift = math.sin(swing_prop * math.pi) * z_clearance
                target_x = x0 + self.vx * stride_scale * (swing_prop - 0.5)
                target_y = y0 + self.vy * stride_scale * (swing_prop - 0.5)
                target_z = z0 + z_lift
            else:
                # STANCE
                stance_prop = (leg_phase - half_cycle) / half_cycle if is_moving else 0.0
                target_x = x0 - self.vx * stride_scale * (stance_prop - 0.5) if is_moving else x0
                target_y = y0 - self.vy * stride_scale * (stance_prop - 0.5) if is_moving else y0
                target_z = z0
                
            # Yaw spin rotation
            if is_moving and abs(self.yaw_rate) > 0.01:
                angle = self.yaw_rate * 0.1 * ((phase % 10) - 5) / 5.0
                c = math.cos(angle)
                s = math.sin(angle)
                tx = target_x * c - target_y * s
                ty = target_x * s + target_y * c
                target_x = tx
                target_y = ty
                
            hip_deg, upper_deg, lower_deg = self.solve_leg_inverse_kinematics(side, sx, sy, target_x, target_y, target_z)
            
            self.joint_sliders[h_idx].set(hip_deg)
            self.joint_sliders[u_idx].set(upper_deg - 90.0)
            self.joint_sliders[l_idx].set(lower_deg)
            self.angles[h_idx] = hip_deg
            self.angles[u_idx] = upper_deg - 90.0
            self.angles[l_idx] = lower_deg
            
        # Push to Arduino
        raw_angles = self.get_raw_servo_angles()
        if self.ser and self.ser.is_open:
            self.push_all_angles(raw_angles)
            
        self.update_3d_view()
        
        # Loop at 50 Hz (20 ms interval)
        if self.gait_active:
            self.gait_after_id = self.after(20, self.gait_step_loop)

    def on_slider_move(self, joint_idx, value):
        # Prevent initialization race condition
        if len(self.invert_vars) < 12 or len(self.joint_sliders) < 12 or len(self.joint_entries) < 12:
            return
            
        val_float = float(value)
        val_int = int(round(val_float))
        thigh_to_calf = {1: 2, 4: 5, 7: 8, 10: 11}
        
        # Sync the Entry widget value text safely
        try:
            if self.focus_get() != self.joint_entries[joint_idx]:
                self.joint_entries[joint_idx].delete(0, tk.END)
                self.joint_entries[joint_idx].insert(0, str(val_int))
        except Exception:
            pass
            
        # Lock knee behavior: move calf slider with thigh slider by the same delta
        if hasattr(self, 'lock_knee') and self.lock_knee.get() and joint_idx in thigh_to_calf:
            calf_idx = thigh_to_calf[joint_idx]
            delta = val_float - self.angles[joint_idx]
            new_calf_val = self.angles[calf_idx] + delta
            new_calf_val = max(10.0, min(170.0, new_calf_val))
            
            # Update thigh angle first to avoid delta mismatch, then trigger calf update
            self.angles[joint_idx] = val_float
            self.joint_sliders[calf_idx].set(new_calf_val)
            
        self.angles[joint_idx] = val_float
        
        # Calculate raw angles dynamically for all joints (including linkage)
        raw_angles = self.get_raw_servo_angles()
        
        # Push all raw physical angles to Arduino since it expects full 16-channel array
        if self.ser and self.ser.is_open:
            self.push_all_angles(raw_angles)
            
        # Redraw 3D Kinematic visualizer
        self.update_3d_view()

    def on_entry_change(self, joint_idx, event):
        try:
            val_str = self.joint_entries[joint_idx].get()
            if not val_str:
                return
            val_float = float(val_str)
            if joint_idx % 3 == 0:     # Hip
                val_float = max(0.0, min(180.0, val_float))
            elif joint_idx % 3 == 1:   # Thigh/Upper
                val_float = max(0.0, min(160.0, val_float))
            else:                      # Calf/Lower
                val_float = max(10.0, min(170.0, val_float))
            self.joint_sliders[joint_idx].set(val_float)
        except ValueError:
            pass

    def adjust_offset(self, joint_idx, amount):
        new_offset = self.offsets[joint_idx] + amount
        # Clamp offsets to +/-15 degrees for upper (thigh) and lower (calf) joints
        if joint_idx % 3 in [1, 2]:
            new_offset = max(-15, min(15, new_offset))
        self.offsets[joint_idx] = new_offset
        self.offset_labels[joint_idx].configure(text=str(new_offset))
        
        # Re-trigger joint slider movement to apply offset immediately
        self.on_slider_move(joint_idx, self.joint_sliders[joint_idx].get())

    def on_inversion_toggle(self, joint_idx):
        self.inversions[joint_idx] = self.invert_vars[joint_idx].get()
        print(f"Joint {joint_idx} inversion set to: {self.inversions[joint_idx]}")
        # Apply updated angle inversion instantly
        self.on_slider_move(joint_idx, self.joint_sliders[joint_idx].get())

    def on_lock_knee_toggle(self):
        # Apply current slider values (this triggers dynamic recompilation and updates everything)
        # We also push the recalculated servo angles to the board if connected
        if self.ser and self.ser.is_open:
            raw_angles = self.get_raw_servo_angles()
            self.push_all_angles(raw_angles)
        self.update_3d_view()

    def trigger_pose(self, pose_name):
        POSES = {
            "Neutral":  [0, 90, 45, 0, 90, 45, 0, 90, 45, 0, 90, 45],
            "Stand":    [0, 130, 60, 0, 130, 60,0, 130, 60, 0, 130, 60],
            "Sit":      [0, 120, 0, 0, 120, 0, 0, 50, 80, 0, 50, 80],
            "Lay Down": [0, 50, 80, 0, 50, 80,0, 50, 80, 0, 50, 80],
            "Assembly": [0, 60,-20, 0, 60,-20, 0, 60,-20, 0, 60,-20]
        }
        
        if pose_name not in POSES:
            return
            
        sw_angles = POSES[pose_name]
        for i in range(12):
            if i % 3 == 1:     # Thigh (direct software angle)
                slider_val = sw_angles[i]
            else:              # Hip and Calf (deviation from 90)
                slider_val = sw_angles[i] + 90.0
                
            # Enforce slider physical boundaries
            if i % 3 == 0:     # Hip
                slider_val = max(0.0, min(180.0, slider_val))
            elif i % 3 == 1:   # Thigh
                slider_val = max(0.0, min(160.0, slider_val))
            else:              # Calf
                slider_val = max(10.0, min(170.0, slider_val))
                
            self.joint_sliders[i].set(slider_val)
            self.angles[i] = slider_val
            
        # Perform multi-joint move on hardware
        if self.ser and self.ser.is_open:
            # Use our linkage-compensated and offset-adjusted raw angles!
            mapped_angles = self.get_raw_servo_angles()
            self.push_all_angles(mapped_angles)
            
        self.update_3d_view()
        print(f"RoboDog commanded to predefined pose: {pose_name}")

    def relax_servos(self):
        if self.ser and self.ser.is_open:
            neutral_angles = [90.0] * 12
            self.push_all_angles(neutral_angles)
            print("RoboDog servos set to neutral 90 degrees (R command not supported by updated Arduino).")
        else:
            messagebox.showwarning("Command Blocked", "Connect a live serial session before relaxing servos.")

    def save_to_pc(self):
        import json
        config_path = os.path.join(os.path.dirname(__file__), "robodog_config.json")
        try:
            # Sync self.inversions list from invert_vars checkboxes
            self.inversions = [bool(var.get()) for var in self.invert_vars]
            
            data = {
                "offsets": self.offsets,
                "inversions": self.inversions,
                "coupling_factors": self.coupling_factors
            }
            with open(config_path, "w") as f:
                json.dump(data, f, indent=4)
            messagebox.showinfo("Saved to PC", f"Calibration and inversion profile saved to PC successfully:\n{config_path}")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save profile to PC:\n{e}")

    def load_from_pc(self, auto_push=True):
        import json
        config_path = os.path.join(os.path.dirname(__file__), "robodog_config.json")
        if not os.path.exists(config_path):
            if not auto_push:  # only warn if user clicked manually
                messagebox.showwarning("File Not Found", f"No calibration profile found at:\n{config_path}")
            return False
            
        try:
            with open(config_path, "r") as f:
                data = json.load(f)
            
            if "offsets" in data:
                self.offsets = list(data["offsets"])
                for i in range(12):
                    if i % 3 in [1, 2]:
                        self.offsets[i] = max(-15, min(15, self.offsets[i]))
            if "inversions" in data:
                self.inversions = list(data["inversions"])
            if "coupling_factors" in data:
                self.coupling_factors = list(data["coupling_factors"])
                for i in range(4):
                    if i < len(self.coupling_vars):
                        self.coupling_vars[i].set(self.coupling_factors[i])
                
            # Sync widgets
            self.sync_offset_labels()
            for i in range(12):
                if i < len(self.invert_vars):
                    self.invert_vars[i].set(self.inversions[i])
            
            # Push calibration offsets live to Arduino if connected
            if auto_push and self.ser and self.ser.is_open:
                self.push_calibration_to_arduino()
                
            if not auto_push:
                messagebox.showinfo("Loaded from PC", f"Profile loaded successfully from PC.")
            return True
        except Exception as e:
            messagebox.showerror("Error", f"Failed to load calibration profile:\n{e}")
            return False

    def push_calibration_to_arduino(self):
        if not self.ser or not self.ser.is_open:
            return
        print("Pushing all active postures (with loaded offsets and linkage compensation) to Arduino Uno...")
        raw_angles = self.get_raw_servo_angles()
        self.push_all_angles(raw_angles)

    def show_matplotlib_warning(self):
        warn_lbl = ttk.Label(
            self.plot_frame,
            text="Matplotlib library not found.\n\nPlease install it using standard terminal commands:\n\npip install matplotlib\n\nTo view interactive 3D kinematic representations.",
            font=("Consolas", 11, "bold"),
            foreground="#ef4444",
            anchor="center",
            justify="center"
        )
        warn_lbl.grid(row=0, column=0, sticky="nsew", padx=20, pady=20)

    def init_3d_plot(self):
        # Configure dark matplotlib style
        plt.style.use('dark_background')
        
        self.fig = plt.figure(facecolor='#1a1a1e')
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Configure canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")
        
        # Grid configuration for the visualizer frame
        self.plot_frame.grid_columnconfigure(0, weight=1)
        self.plot_frame.grid_rowconfigure(0, weight=1)

    def get_leg_forward_kinematics(self, side, dx, dy, hip_angle, upper_angle, lower_angle):
        # Angles to radians, zero frame mappings
        # In typical IK layouts, zero degrees is aligned neutrally:
        theta_hip = math.radians(hip_angle - 90.0)
        theta_upper = math.radians(upper_angle)
        theta_lower = math.radians(lower_angle - 90.0)
        
        # 1. Hip Abduction/Adduction End Effector
        # The Hip pivots outwards depending on left/right side configuration
        hx = dx
        hy = dy + side * L_HIP * math.cos(theta_hip)
        hz = L_HIP * math.sin(theta_hip)
        
        # 2. Knee coordinate (Upper leg segment)
        # Pitch rotation determines motion path in X/Z plane projected with Roll angle
        u_dx = L_UPPER * math.sin(theta_upper)
        u_dz = -L_UPPER * math.cos(theta_upper)
        
        kx = hx + u_dx
        ky = hy - u_dz * math.sin(theta_hip)
        kz = hz + u_dz * math.cos(theta_hip)
        
        # 3. Foot end effector (Lower leg segment)
        # In a parallel lever linkage mechanism, the absolute angle of the calf in space
        # is directly driven by the lower motor independently of the thigh angle!
        l_dx = u_dx + L_LOWER * math.sin(theta_lower)
        l_dz = u_dz - L_LOWER * math.cos(theta_lower)
        
        fx = hx + l_dx
        fy = hy - l_dz * math.sin(theta_hip)
        fz = hz + l_dz * math.cos(theta_hip)
        
        return (hx, hy, hz), (kx, ky, kz), (fx, fy, fz)

    def update_3d_view(self):
        if not matplotlib or not hasattr(self, 'ax'):
            return
            
        self.ax.clear()
        
        # Set standard natural 3D perspective where Z (Height) is vertical
        self.ax.view_init(elev=25, azim=-60)
        
        # Plot styling limits
        self.ax.set_xlim(-0.25, 0.25)
        self.ax.set_ylim(-0.20, 0.20)
        self.ax.set_zlim(-0.25, 0.15)
        
        # Labels and design setup (perfectly aligned with physical robot coords)
        self.ax.set_xlabel("X (Forward/Back)", color="#e4e4e7", fontname="Consolas", fontsize=11)
        self.ax.set_ylabel("Y (Left/Right)", color="#e4e4e7", fontname="Consolas", fontsize=11)
        self.ax.set_zlabel("Z (Height)", color="#e4e4e7", fontname="Consolas", fontsize=11)
        self.ax.set_title("RoboDog 3D Live Simulation View", color="#38bdf8", fontname="Consolas", fontsize=14, fontweight="bold")
        self.ax.tick_params(colors="#e4e4e7", labelsize=10)
        
        # Dark grid elements
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.ax.grid(True, color="#2d2d34", linestyle=":")
        
        # Torso boundaries based on shoulder attachments
        fr_shoulder = (BODY_LENGTH/2, -BODY_WIDTH/2, 0)
        fl_shoulder = (BODY_LENGTH/2, BODY_WIDTH/2, 0)
        br_shoulder = (-BODY_LENGTH/2, -BODY_WIDTH/2, 0)
        bl_shoulder = (-BODY_LENGTH/2, BODY_WIDTH/2, 0)
        
        # 1. Plot Torso Box
        torso_x = [fr_shoulder[0], fl_shoulder[0], bl_shoulder[0], br_shoulder[0], fr_shoulder[0]]
        torso_y = [fr_shoulder[1], fl_shoulder[1], bl_shoulder[1], br_shoulder[1], fr_shoulder[1]]
        torso_z = [0, 0, 0, 0, 0]
        self.ax.plot(torso_x, torso_y, torso_z, color="#38bdf8", linewidth=3, label="Main Chassis")
        
        # Add persistent 3D direction text markers that rotate dynamically with the perspective
        self.ax.text(BODY_LENGTH/2 + 0.04, 0, 0.01, "FRONT (Fwd)", color="#38bdf8", fontname="Consolas", fontsize=10, fontweight="bold", ha="center")
        self.ax.text(-BODY_LENGTH/2 - 0.04, 0, 0.01, "BACK (Bwd)", color="#ef4444", fontname="Consolas", fontsize=10, fontweight="bold", ha="center")
        self.ax.text(0, BODY_WIDTH/2 + 0.04, 0.01, "LEFT", color="#a855f7", fontname="Consolas", fontsize=10, fontweight="bold", ha="center")
        self.ax.text(0, -BODY_WIDTH/2 - 0.04, 0.01, "RIGHT", color="#10b981", fontname="Consolas", fontsize=10, fontweight="bold", ha="center")

        # Get all 12 raw angles for monitor display
        raw_angles = self.get_raw_servo_angles()

        # 2. Leg 3D Kinematics Mapping
        # Leg index sequence: 0:FR, 1:FL, 2:BR, 3:BL
        legs_config = [
            (-1, fr_shoulder[0], fr_shoulder[1], 0, 1, 2, "#10b981", "FR Leg", 0), # FR
            (1, fl_shoulder[0], fl_shoulder[1], 3, 4, 5, "#a855f7", "FL Leg", 1),  # FL
            (-1, br_shoulder[0], br_shoulder[1], 6, 7, 8, "#f59e0b", "BR Leg", 2), # BR
            (1, bl_shoulder[0], bl_shoulder[1], 9, 10, 11, "#ec4899", "BL Leg", 3) # BL
        ]
        
        for side, sx, sy, h_idx, u_idx, l_idx, color, leg_name, leg_idx in legs_config:
            hip_ang = self.angles[h_idx]
            up_ang = self.angles[u_idx]
            low_ang = self.angles[l_idx]
            
            h_pt, k_pt, f_pt = self.get_leg_forward_kinematics(side, sx, sy, hip_ang, up_ang, low_ang)
            
            # Draw hip joint line
            self.ax.plot([sx, h_pt[0]], [sy, h_pt[1]], [0, h_pt[2]], color="#e4e4e7", linewidth=2, linestyle="--")
            # Draw upper leg link
            self.ax.plot([h_pt[0], k_pt[0]], [h_pt[1], k_pt[1]], [h_pt[2], k_pt[2]], color=color, linewidth=4)
            # Draw lower leg link
            self.ax.plot([k_pt[0], f_pt[0]], [k_pt[1], f_pt[1]], [k_pt[2], f_pt[2]], color=color, linewidth=4)
            
            # Plot joints circles
            self.ax.scatter([sx, h_pt[0], k_pt[0]], [sy, h_pt[1], k_pt[1]], [0, h_pt[2], k_pt[2]], color="#ffffff", s=15, zorder=5)
            # Plot contact footpad
            self.ax.scatter([f_pt[0]], [f_pt[1]], [f_pt[2]], color="#ef4444", s=30, zorder=5)

            # Update the Monitor Dashboard labels for this leg
            if hasattr(self, 'leg_monitor_labels') and leg_idx in self.leg_monitor_labels:
                # 1. Cartesian position relative to body center mapped to user coords
                # Consistent with standard robotics and 3D visualizer labels:
                # Forward/Back is User_X (Front is +X, Back is -X) -> X axis in visualizer
                # Left/Right is User_Y (Left is +Y, Right is -Y) -> Y axis in visualizer
                # Height is User_Z (Up is +Z, Down is -Z) -> Z axis in visualizer
                user_x = f_pt[0] * 100.0  # cm (X axis)
                user_y = f_pt[1] * 100.0  # cm (Y axis)
                user_z = f_pt[2] * 100.0  # cm (Z axis)
                
                # Relative to hip attachment (X is Forward/Back, Y is Left/Right)
                rel_x = (f_pt[0] - sx) * 100.0
                rel_y = (f_pt[1] - sy) * 100.0
                
                pos_text = (f"Pos (cm):\n"
                            f"  X (Fwd): {user_x:+.1f} (rel {rel_x:+.1f})\n"
                            f"  Y (L/R): {user_y:+.1f} (rel {rel_y:+.1f})\n"
                            f"  Z (Hgt): {user_z:+.1f}")
                self.leg_monitor_labels[leg_idx]["pos"].configure(text=pos_text)
                
                # 2. Software Angles (deviations from neutral 90 degrees)
                sw_hip = hip_ang - 90.0
                sw_thigh = up_ang
                sw_calf = low_ang - 90.0
                sw_text = (f"SW Angles:\n"
                           f"  Hip:   {sw_hip:+.1f}°\n"
                           f"  Thigh: {sw_thigh:+.1f}°\n"
                           f"  Calf:  {sw_calf:+.1f}°")
                self.leg_monitor_labels[leg_idx]["sw"].configure(text=sw_text)
                
                # 3. Raw/Real Servo outputs (rounded to nearest integer for the hardware)
                r_h = int(round(raw_angles[h_idx]))
                r_u = int(round(raw_angles[u_idx]))
                r_l = int(round(raw_angles[l_idx]))
                raw_text = (f"Servo Outputs:\n"
                            f"  Hip:   {r_h:3d}°\n"
                            f"  Thigh: {r_u:3d}°\n"
                            f"  Calf:  {r_l:3d}°")
                self.leg_monitor_labels[leg_idx]["raw"].configure(text=raw_text)

        # Legend representation
        self.ax.legend(loc="upper right", fontsize=10, framealpha=0.5)
        self.fig.tight_layout()
        self.canvas.draw_idle()

    def on_close(self):
        print("Closing gracefully...")
        # 1. Cancel background loop timers to prevent invalid command callbacks after window destruction
        if self.dither_after_id:
            self.after_cancel(self.dither_after_id)
        if self.gait_after_id:
            self.after_cancel(self.gait_after_id)
            
        # 2. Set servos to neutral and close serial session cleanly
        if self.ser and self.ser.is_open:
            try:
                neutral_angles = [90.0] * 12
                self.push_all_angles(neutral_angles)
                time.sleep(0.1)
                self.ser.close()
                print("Motors set to neutral and serial port closed cleanly.")
            except Exception as e:
                print(f"Error during graceful serial shutdown: {e}")
                
        # 3. Destroy GUI window
        self.destroy()

if __name__ == "__main__":
    app = RoboDogGuiApp()
    app.mainloop()
