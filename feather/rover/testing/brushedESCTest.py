import board
import pwmio
import digitalio
import time

# Set up PWM pin at 50Hz for ESC
esc = pwmio.PWMOut(board.D5, frequency=50)

# Set up encoder pins
enc_a = digitalio.DigitalInOut(board.D11)
enc_a.direction = digitalio.Direction.INPUT
enc_a.pull = digitalio.Pull.UP

enc_b = digitalio.DigitalInOut(board.D12)
enc_b.direction = digitalio.Direction.INPUT
enc_b.pull = digitalio.Pull.UP

# Encoder constants
CPR = 48  # counts per revolution (48 CPR encoder)
GEAR_RATIO = 34.014  # gear ratio of the motor
COUNTS_PER_OUTPUT_REV = CPR * GEAR_RATIO  # total counts per output shaft revolution

# Encoder tracking variables
encoder_count = 0
last_a_state = enc_a.value

def set_pulse(pulse_us):
    duty = int((pulse_us / 20000) * 65535)
    esc.duty_cycle = duty

def read_encoder():
    global encoder_count, last_a_state
    current_a = enc_a.value
    if current_a != last_a_state:
        if enc_b.value != current_a:
            encoder_count += 1
        else:
            encoder_count -= 1
    last_a_state = current_a

def get_rpm(counts, elapsed_time):
    # Convert counts over elapsed time to RPM at output shaft
    revolutions = counts / COUNTS_PER_OUTPUT_REV
    minutes = elapsed_time / 60
    return revolutions / minutes

def run_speed_test(pulse_us, duration, label):
    global encoder_count
    print(f"\n{label}")
    set_pulse(pulse_us)
    
    start_time = time.monotonic()
    last_print_time = start_time
    last_count = encoder_count
    
    while time.monotonic() - start_time < duration:
        read_encoder()
        
        # Print RPM every 0.5 seconds
        current_time = time.monotonic()
        if current_time - last_print_time >= 0.5:
            elapsed = current_time - last_print_time
            counts = abs(encoder_count - last_count)
            rpm = get_rpm(counts, elapsed)
            print(f"  RPM: {rpm:.1f}  |  Total counts: {encoder_count}")
            last_print_time = current_time
            last_count = encoder_count

# ── Arming sequence ──────────────────────────────────────
print("Starting ESC arming sequence...")
set_pulse(2000)
time.sleep(2)
set_pulse(1000)
time.sleep(2)
print("ESC armed!")

# ── Speed tests ──────────────────────────────────────────
run_speed_test(1100, 4, "Low speed (1100us)...")
run_speed_test(1300, 4, "Low-medium speed (1300us)...")
run_speed_test(1500, 4, "Medium speed (1500us)...")

# ── Stop ─────────────────────────────────────────────────
print("\nStopping motor...")
set_pulse(1000)
print("Test complete")