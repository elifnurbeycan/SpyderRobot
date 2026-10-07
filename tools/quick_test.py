#!/usr/bin/env python3
"""
quick_test.py — Hızlı Donanım ve Motor Testi
Batarya açıkken 3 saniye içinde motor haberleşmesini ve hareketini test eder.
"""
import sys
import time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

def main():
    print(f"[{PORT}] Bağlanıyor...")
    try:
        s = serial.Serial(PORT, 115200, timeout=1.0)
    except Exception as e:
        print(f"HATA: Seri port açılamadı ({e})")
        return

    time.sleep(2.0)  # Arduino reset bekle
    s.reset_input_buffer()

    print("\n--- 1. MOTOR POZİSYON SORGUSU (ID 1-18) ---")
    s.write(b"A\n")
    time.sleep(0.5)

    ok_count = 0
    t0 = time.time()
    while time.time() - t0 < 3.0:
        line = s.readline().decode('utf-8', errors='replace').strip()
        if line:
            print("  ", line)
            if line.startswith("P ") and not line.endswith("-1"):
                ok_count += 1
        if "A DONE" in line:
            break

    if ok_count > 0:
        print(f"\n✅ BAŞARILI! {ok_count} motor canlı cevap verdi.")
        print("\n--- 2. CANLI HAREKET TESTİ (Femur ID 2 & Tibia ID 1) ---")
        print("  Femur ID 2 -> 3283 pozisyonuna gönderiliyor...")
        s.write(b"M 2 3283\n")
        time.sleep(0.3)
        print("  Tibia ID 1 -> 465 pozisyonuna gönderiliyor...")
        s.write(b"M 1 465\n")
        time.sleep(0.3)
        print("✅ HAREKET KOMUTLARI İLETİLDİ.")
    else:
        print("\n❌ UYARI: Henüz motorlardan yanıt gelmedi (-1). Güç anahtarını kontrol edin.")

    s.close()

if __name__ == "__main__":
    main()
