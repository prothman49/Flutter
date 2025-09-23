from Phidget22.Devices.VoltageOutput import VoltageOutput
from Phidget22.PhidgetException import PhidgetException

try:
    vo = VoltageOutput()
    vo.setChannel(0)     # Always 0 for OUT1000
    vo.setHubPort(0)     # Your hub port number (0 or 1)
    # Do NOT call vo.setIsHubPortDevice(True) here
    vo.openWaitForAttachment(5000)

    vo.setVoltage(3.0)
    print("[INFO] Voltage set to 3.0V")
    input("Press Enter to exit...")
    vo.setVoltage(0.0)
    vo.close()
except PhidgetException as e:
    print(f"Phidget error: {e}")

