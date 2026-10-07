/*
 * mega_full_bridge.cpp  –  ST3020 + AX-12A çift seri köprü
 *
 * ST3020  : Serial1 (TX=18 RX=19) @ 1 Mbaud  |  IDs: 2 5 8 11 14 17
 * AX-12A  : Serial2 (TX=16 RX=17) @ 1 Mbaud  |  IDs: 1 3 4 6 7 9 10 12 13 15 16 18
 *
 * Komut formatı (PC'den, \n ile biter):
 *   M <id> <enc>    – motoru encoder pozisyonuna götür
 *   A               – tüm 18 motorun pozisyonunu oku
 *
 * Yanıtlar:
 *   OK              – M komutu kabul edildi
 *   P <id> <pos>    – pozisyon değeri
 *   A DONE          – A komutu tamamlandı
 *   READY           – başlangıç
 */

#include <Arduino.h>

#define ST_SERIAL  Serial1   // ST3020
#define AX_SERIAL  Serial2   // AX-12A
#define BAUD       1000000UL

#define INST_READ   0x02
#define INST_WRITE  0x03

// ST3020 adresleri
#define ST_GOAL_POS   42   // 2 byte  (+ 2 byte time + 2 byte speed = 7 param)
#define ST_PRESENT    56   // 2 byte
#define ST_TORQUE     40   // 1 byte  torque enable

// AX-12A adresleri
#define AX_GOAL_POS   30   // 2 byte
#define AX_PRESENT    36   // 2 byte
#define AX_TORQUE     24   // 1 byte  torque enable

// Motor ID → tip tablosu: true = ST3020, false = AX-12A
const uint8_t ST_IDS[] = {2, 5, 8, 11, 14, 17};
const uint8_t AX_IDS[] = {1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18};
const uint8_t ALL_IDS[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
const uint8_t MOTOR_COUNT = 18;

bool isST(uint8_t id) {
    for (uint8_t i = 0; i < 6; i++) if (ST_IDS[i] == id) return true;
    return false;
}
HardwareSerial& serFor(uint8_t id) { return isST(id) ? ST_SERIAL : AX_SERIAL; }

// ── Paket gönder ────────────────────────────────────────────────────────────
void sendPkt(HardwareSerial &ser, uint8_t id, uint8_t inst, uint8_t *p, uint8_t pLen) {
    uint8_t len = pLen + 2, cs = id + len + inst;
    ser.write(0xFF); ser.write(0xFF);
    ser.write(id);   ser.write(len);
    ser.write(inst);
    for (uint8_t i = 0; i < pLen; i++) { ser.write(p[i]); cs += p[i]; }
    ser.write((uint8_t)(~cs));
    ser.flush();
}

// ── ST3020 pozisyona git ─────────────────────────────────────────────────────
void stMove(uint8_t id, uint16_t pos) {
    if (pos > 4095) pos = 4095;
    uint8_t p[7] = { ST_GOAL_POS,
                     (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
                     0, 0, 200 & 0xFF, 200 >> 8 };
    sendPkt(ST_SERIAL, id, INST_WRITE, p, 7);
}

// ── AX-12A pozisyona git ─────────────────────────────────────────────────────
void axMove(uint8_t id, uint16_t pos) {
    if (pos > 1023) pos = 1023;
    uint8_t p[3] = { AX_GOAL_POS,
                     (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8) };
    sendPkt(AX_SERIAL, id, INST_WRITE, p, 3);
}

// ── Pozisyon oku ─────────────────────────────────────────────────────────────
//  TX echo + yanit
int16_t readPos(uint8_t id) {
    HardwareSerial &ser = serFor(id);
    uint8_t addr = isST(id) ? ST_PRESENT : AX_PRESENT;

    while (ser.available()) ser.read();
    uint8_t p[2] = { addr, 2 };
    sendPkt(ser, id, INST_READ, p, 2);

    uint8_t buf[24]; uint8_t idx = 0;
    unsigned long t = millis();
    while (idx < 24 && millis() - t < 40)
        if (ser.available()) buf[idx++] = ser.read();
    if (idx < 13) return -1;
    return (int16_t)((uint16_t)buf[idx - 3] | ((uint16_t)buf[idx - 2] << 8));
}

// ── Torku ac (goal position yazmadan once sart) ──────────────────────────────
void torqueOn(uint8_t id) {
    if (isST(id)) { uint8_t p[2] = { ST_TORQUE, 1 }; sendPkt(ST_SERIAL, id, INST_WRITE, p, 2); }
    else          { uint8_t p[2] = { AX_TORQUE, 1 }; sendPkt(AX_SERIAL, id, INST_WRITE, p, 2); }
}

// ── Setup / Loop ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    ST_SERIAL.begin(BAUD);
    AX_SERIAL.begin(BAUD);
    delay(500);
    // Tum motorlarin torkunu ac
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) { torqueOn(ALL_IDS[i]); delay(15); }
    Serial.println(F("READY"));
}

void loop() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (!cmd.length()) return;

    int s1 = cmd.indexOf(' ');
    String verb = (s1 < 0) ? cmd : cmd.substring(0, s1);
    verb.toUpperCase();

    if (verb == "M") {
        // M <id> <enc>
        String rest = cmd.substring(s1 + 1);
        int s2 = rest.indexOf(' ');
        uint8_t  id  = rest.substring(0, s2).toInt();
        uint16_t pos = rest.substring(s2 + 1).toInt();
        if (isST(id)) stMove(id, pos);
        else          axMove(id, pos);
        Serial.println(F("OK"));
    }
    else if (verb == "T") {
        // T [id] — tork ac
        if (s1 > 0) {
            uint8_t id = cmd.substring(s1 + 1).toInt();
            torqueOn(id);
        } else {
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) { torqueOn(ALL_IDS[i]); delay(10); }
        }
        Serial.println(F("OK"));
    }
    else if (verb == "A") {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            int16_t p = readPos(ALL_IDS[i]);
            Serial.print(F("P ")); Serial.print(ALL_IDS[i]);
            Serial.print(' ');     Serial.println(p);
        }
        Serial.println(F("A DONE"));
    }
    else if (verb == "R") {
        // R <id> — tek motorun pozisyonunu oku
        uint8_t id = cmd.substring(s1 + 1).toInt();
        int16_t p = readPos(id);
        Serial.print(F("P ")); Serial.print(id);
        Serial.print(' ');     Serial.println(p);
    }
}
