#!/usr/bin/env python3
"""
test_joints.py — Eklemleri Tek Tek Oynatma Testi
18 motoru gruplar ve sirayla hareket ettirerek fiziki hareketi dogrular.
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

ST3020_STANCE = { 2: 3283, 5: 1918, 8: 3616, 11: 2997, 14: 3716, 17: 2994 }
AX12_STANCE   = { 1: 465, 3: 563, 4: 699, 6: 527, 7: 403, 9: 518, 10: 821, 12: 593, 13: 1023, 15: 908, 16: 496, 18: 303 }

def main():
    print(f"Port baglaniyor: {PORT}...")
    s = serial.Serial(PORT, 115200, timeout=1.0)
    time.sleep(2.0)
    s.reset_input_buffer()

    print("\n1. Tum motorlar STANCE pozisyonuna aliniyor...")
    for m_id, pos in ST3020_STANCE.items():
        s.write(f"M {m_id} {pos}\n".encode())
        time.sleep(0.02)
    for m_id, pos in AX12_STANCE.items():
        s.write(f"M {m_id} {pos}\n".encode())
        time.sleep(0.02)
    time.sleep(1.0)

    print("\n2. FEMUR MOTORLARI (ST3020) SALINIM TESTI...")
    # Femurlari sirayla -200/+200 adim oynat
    for m_id, pos in ST3020_STANCE.items():
        print(f"  -> Femur ID {m_id} oynatiliyor...")
        s.write(f"M {m_id} {pos - 200}\n".encode())
        time.sleep(0.5)
        s.write(f"M {m_id} {pos + 200}\n".encode())
        time.sleep(0.5)
        s.write(f"M {m_id} {pos}\n".encode())
        time.sleep(0.3)

    print("\n3. COXA MOTORLARI (AX-12A) SALINIM TESTI...")
    COXA_IDS = [3, 6, 9, 10, 13, 16]
    for m_id in COXA_IDS:
        pos = AX12_STANCE[m_id]
        print(f"  -> Coxa ID {m_id} oynatiliyor...")
        s.write(f"M {m_id} {pos - 100}\n".encode())
        time.sleep(0.5)
        s.write(f"M {m_id} {pos + 100}\n".encode())
        time.sleep(0.5)
        s.write(f"M {m_id} {pos}\n".encode())
        time.sleep(0.3)

    s.close()
    print("\nTest bitti.")

if __name__ == "__main__":
    main()
