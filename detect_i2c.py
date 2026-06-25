#!/usr/bin/env python3
import sys

def scan_using_smbus():
    print("Attempting to scan I2C bus 1 using smbus/smbus2...")
    try:
        import smbus2
        bus = smbus2.SMBus(1)
        use_smbus2 = True
    except ImportError:
        try:
            import smbus
            bus = smbus.SMBus(1)
            use_smbus2 = False
        except ImportError:
            print("Error: Neither 'smbus2' nor 'smbus' library is installed.")
            return False

    detected = []
    # Address range 0x03 to 0x77
    for addr in range(0x03, 0x78):
        try:
            if use_smbus2:
                bus.write_quick(addr)
            else:
                # write_quick equivalent for older smbus
                bus.write_byte(addr, 0)
            detected.append(addr)
        except OSError:
            pass
            
    if detected:
        print(f"Success! Detected devices: {', '.join([hex(a) for a in detected])}")
    else:
        print("Scan complete. No I2C devices detected on Bus 1.")
    return True

def scan_using_circuitpython():
    print("Attempting to scan using Adafruit Board/Busio...")
    try:
        import board
        import busio
    except ImportError:
        print("Error: CircuitPython libraries ('board' or 'busio') are not installed.")
        return False

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"Error initializing I2C bus: {e}")
        return False

    while not i2c.try_lock():
        pass
        
    try:
        devices = i2c.scan()
        if devices:
            print(f"Success! Detected devices: {', '.join([hex(a) for a in devices])}")
        else:
            print("Scan complete. No I2C devices detected.")
    finally:
        i2c.unlock()
    return True

def main():
    print("=== RoboDog Quadruped - I2C Scanner ===")
    # Try smbus first
    success = scan_using_smbus()
    if not success:
        # Fallback to CircuitPython
        scan_using_circuitpython()

if __name__ == "__main__":
    main()
