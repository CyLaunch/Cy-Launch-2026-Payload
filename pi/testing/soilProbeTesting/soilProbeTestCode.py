# ======================================================================================
# Developer: 
# Program: Test soil probe data collection
# Contact: 
# ======================================================================================


import minimalmodbus
import serial
import serial.tools.list_ports
import platform

def create_instrument(port, slave_address=1):
    """
    Create and configure a minimalmodbus instrument.
    """
    instrument = minimalmodbus.Instrument(port, slave_address)
    instrument.serial.baudrate = 9600
    instrument.serial.bytesize = 8
    instrument.serial.parity   = serial.PARITY_NONE
    instrument.serial.stopbits = 1
    instrument.serial.timeout  = 1
    instrument.mode = minimalmodbus.MODE_RTU
    instrument.serial.rtscts = False
    return instrument

def test_port(port):
    """
    Try to connect and read a known register to see if the device is present.
    Returns the instrument if successful, None otherwise.
    """
    try:
        instrument = create_instrument(port)
        # Test read PH register (address 6)
        _ = instrument.read_register(6, 0, functioncode=3, signed=False)
        return instrument
    except Exception:
        return None

def find_device():
    """
    Scan all serial ports and return the first one that responds.
    """
    ports = serial.tools.list_ports.comports()
    for port in ports:
        instrument = test_port(port.device)
        if instrument:
            print(f"Device found on port: {port.device}")
            return instrument
    return None

def read_soil_probe(instrument):
    """
    Read PH, Soil Nitrogen, and Soil Conductivity.
    """
    try:
        ph_raw = instrument.read_register(6, 0, functioncode=3, signed=False)
        print(f"PH Raw Value: {ph_raw}")

        nitrogen_regs = instrument.read_registers(30, 3, functioncode=3)
        print(f"Soil Nitrogen Raw Registers: {nitrogen_regs}")

        conductivity_raw = instrument.read_register(21, 0, functioncode=3, signed=False)
        print(f"Soil Conductivity Raw Value: {conductivity_raw}")

    except Exception as e:
        print(f"Error reading sensor: {e}")

def main():
    print(f"Detected OS: {platform.system()}")

    instrument = find_device()
    if not instrument:
        print("No device found on any COM port.")
        return

    read_soil_probe(instrument)

if __name__ == "__main__":
    main()
