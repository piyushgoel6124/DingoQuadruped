import tkinter as tk
from tkinter import ttk
import serial
import threading
import time

PORT = "COM6"
BAUD = 115200

ser = serial.Serial(PORT, BAUD)

angles = [90] * 16
last_sent = ""


def tx_thread():
    global last_sent

    while True:
        packet = ",".join(map(str, angles))

        if packet != last_sent:
            try:
                ser.write((packet + "\n").encode())
                last_sent = packet
            except:
                pass

        time.sleep(0.05)


def slider_changed(channel, value):
    value = round(float(value))

    if value != angles[channel]:
        angles[channel] = value
        labels[channel]["text"] = f"{value}"


root = tk.Tk()
root.title("16 Servo Controller")

frame = tk.Frame(root)
frame.pack(padx=10, pady=10)

labels = []

for ch in range(16):

    col = tk.Frame(frame)
    col.grid(row=0, column=ch)

    lbl = tk.Label(col, text="90")
    lbl.pack()

    labels.append(lbl)

    slider = ttk.Scale(
        col,
        from_=180,
        to=0,
        orient="vertical",
        length=500,
        command=lambda v, c=ch: slider_changed(c, v)
    )

    slider.set(90)
    slider.pack()

threading.Thread(
    target=tx_thread,
    daemon=True
).start()

root.mainloop()