#!/usr/bin/env python3
# ID 14'e encoder hedefi ver. Sayi yaz + Enter -> motor oraya gider.
# Kullanim: python3 g14.py [/dev/ttyUSB0]     cikis: q
import sys, time, serial
port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
s = serial.Serial(port, 115200, timeout=1)
time.sleep(2.5); s.reset_input_buffer()
print("14'e hedef ver. Sayi yaz + Enter. Cikis: q")
while True:
    v = input("14> ").strip()
    if v.lower() == "q":
        break
    if not v.lstrip("-").isdigit():
        continue
    s.reset_input_buffer()
    s.write(f"g {v}\n".encode())      # firmware: izle(14, hedef) -> present akisi
    time.sleep(5.2)
    print(s.read(s.in_waiting or 1).decode(errors="replace"))
s.close()
