#!/usr/bin/env python3
"""
jog14.py — ID 14'e klavyeden encoder hedefi ver, present'i gozlemle.

Firmware: mega_st3020_mode.cpp  (komut: j <hedef> -> "hedef=.. present=..")

Kullanim:
    python3 jog14.py [/dev/ttyUSB0]

Komutlar (yaz + Enter):
    <sayi>   -> 14'u o encoder degerine gonder      (or: 3812)
    +[n]     -> mevcut hedefe +n ekle (n yoksa +50)  (or: +  veya  +100)
    -[n]     -> mevcut hedeften -n cikar             (or: -  veya  -100)
    p        -> sadece present pozisyonu oku (hareket yok)
    q        -> cikis

Her komuttan sonra hedef ve present ekrana yazilir; ayni zamanda
jog14_log.csv'ye (hedef,present) kaydedilir.
"""
import sys, time, serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
STEP_DEFAULT = 50

def read_reply(s, timeout=1.0):
    t0 = time.time()
    line = b""
    while time.time() - t0 < timeout:
        b = s.read(1)
        if b == b"\n":
            break
        if b:
            line += b
    return line.decode(errors="replace").strip()

def send_target(s, hedef):
    s.reset_input_buffer()
    s.write(f"j {hedef}\n".encode())
    # firmware ~350ms bekliyor, yanit sonra gelir
    time.sleep(0.4)
    return read_reply(s, timeout=1.2)

def main():
    s = serial.Serial(PORT, 115200, timeout=1)
    time.sleep(2.5)
    s.reset_input_buffer()
    print(f"Baglandi: {PORT}")
    print("Komut: <sayi>=hedef  +/-[n]=adim  p=oku  q=cikis")
    print("Ornek: 3812  |  +100  |  -  |  p\n")

    log = open("jog14_log.csv", "w")
    log.write("hedef,present\n"); log.flush()

    hedef = None
    while True:
        try:
            cmd = input("14> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not cmd:
            continue
        if cmd.lower() == "q":
            break

        if cmd.lower() == "p":
            s.reset_input_buffer()
            # present okumak icin mevcut hedefi tekrar gonder (yoksa 2048)
            h = hedef if hedef is not None else 2048
            reply = send_target(s, h)
            print("  ", reply)
            continue

        if cmd[0] in "+-":
            if hedef is None:
                print("  once bir sayi ile hedef ver."); continue
            n = cmd[1:].strip()
            delta = int(n) if n else STEP_DEFAULT
            hedef += delta if cmd[0] == "+" else -delta
        else:
            try:
                hedef = int(cmd)
            except ValueError:
                print("  gecersiz. sayi/+/-/p/q yaz."); continue

        hedef = max(0, min(4095, hedef))
        reply = send_target(s, hedef)
        print("  ", reply)

        # loga present'i ayikla
        present = ""
        if "present=" in reply:
            present = reply.split("present=")[1].strip()
        log.write(f"{hedef},{present}\n"); log.flush()

    log.close()
    s.close()
    print("\nCikis. Kayit: jog14_log.csv")

if __name__ == "__main__":
    main()
