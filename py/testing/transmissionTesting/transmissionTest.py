# ======================================================================================
# Developer: Noah Wons
# Program: Test transmitter communication with featherwing extra m4 and raspi
# Contact: wons123@iastate.edu
# ======================================================================================


import board
import busio
import digitalio
import adafruit_rfm69

# Pin definitions (match your physical wiring)
CS = digitalio.DigitalInOut(board.CE1)    # Pin 26
RESET = digitalio.DigitalInOut(board.D25) # Pin 22
FREQ = 915.0 # Set to 433.0 or 915.0 depending on your board

# Initialize SPI
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Attempt to initialize the RFM69 radio
try:
    rfm69 = adafruit_rfm69.RFM69(spi, CS, RESET, FREQ)
    # RFM69 often requires an encryption key to talk to other radios
    rfm69.encryption_key = b'\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08'
    print("RFM69 radio detected!")
    # Simple Send loop
    while True:
        print("Sending packet...")
        rfm69.send(bytes("Hello from Pi!\r\n", "utf-8"))
        
        # Check for a response
        packet = rfm69.receive(timeout=2.0)
        if packet:
            print(f"Received: {packet}")
except RuntimeError as e:
    print(f"Radio not found: {e}. Check your CS and Reset wiring.")


