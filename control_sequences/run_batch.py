import csv
import subprocess
import tempfile
import os
import sys

# === Command-line argument for sequence type ===
if len(sys.argv) < 2:
    print("Usage: python run_sequences.py <sequence_type>")
    print("Example: python run_sequences.py step")
    sys.exit(1)

SEQUENCE_TYPE = sys.argv[1]
if SEQUENCE_TYPE not in {"step", "ramp"}:
    print("Error: sequence_type must be 'step' or 'ramp'")
    sys.exit(1)

# === Hardcoded configuration ===
SEND_CAN_ID = 7
CAN_INTERFACE = "can0"
SAMPLE_RATE_HZ = 2000
EXECUTABLE = f"./build/{SEQUENCE_TYPE}"
INPUT_DIR = f"control_sequences/inputs/{SEQUENCE_TYPE}"
CSV_FILE = f"{INPUT_DIR}/motor{SEND_CAN_ID}.csv"

# === Ensure input directory exists ===
os.makedirs(INPUT_DIR, exist_ok=True)

# === Main loop ===
with open(CSV_FILE, newline="") as csvfile:
    reader = csv.DictReader(csvfile)
    for i, row in enumerate(reader, start=1):
        test_name = row.get("test_name", f"test{i}")

        # Create a temporary input file in the input directory
        with tempfile.NamedTemporaryFile(mode="w", dir=INPUT_DIR, suffix=".in", delete=False) as tmpfile:
            input_path = tmpfile.name
            tmpfile.write(f"send_can_id = {SEND_CAN_ID}\n")
            tmpfile.write(f"can_interface = {CAN_INTERFACE}\n\n")

            # --- Handle step vs trap ---
            if SEQUENCE_TYPE == "step":
                tmpfile.write(f"step_torque = {row['step_torque']}\n")
                tmpfile.write(f"step_duration = {row['step_duration']}\n")
            else:  # trap sequence
                tmpfile.write(f"rise_width = {row['rise_width']}\n")
                tmpfile.write(f"plateau_width = {row['plateau_width']}\n")
                tmpfile.write(f"fall_width = {row['fall_width']}\n")
                tmpfile.write(f"max_torque = {row['max_torque']}\n")

            tmpfile.write(f"\nresolution = {SAMPLE_RATE_HZ}\n")
            tmpfile.write(f"test_name = {test_name}\n")

        print(f"→ Running {SEQUENCE_TYPE} test {i}: {test_name}")
        try:
            subprocess.run([EXECUTABLE, input_path], check=True)
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Test {i} failed: {e}")
        finally:
            os.remove(input_path)
