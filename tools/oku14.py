#!/usr/bin/env python3
# ID 14 torku kapatir, sonra present encoder'i CANLI ekrana basar.
# Sen motoru ELLE cevir, deger degisimini gor.
# Kullanim: python3 oku14.py [/dev/ttyUSB0]     cikis: Ctrl+C
import sys, time, serial
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
s = serial.Serial(port, 115200, timeout=0.3)
time.sleep(2.5); s.reset_input_buffer()

s.write(b"d\n")            # torku kapat -> motor serbest
time.sleep(0.3); s.reset_input_buffer()
print("Torque OFF. Motoru ELLE cevir. Cikis: Ctrl+C\n")

try:
    while True:
        s.reset_input_buffer()
        s.write(b"k\n")                       # present oku
        line = s.readline().decode(errors="replace").strip()
        if line.lstrip("-").isdigit():
            print(f"\r  present = {int(line):5d}   ", end="", flush=True)
        time.sleep(0.1)                       # ~10 Hz
except KeyboardInterrupt:
    print("\nCikis.")
    s.close()
