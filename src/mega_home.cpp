/*
 * mega_home.cpp - 18 motoru olculmus MERKEZ pozisyonuna goturur (home dusus)
 *
 * motor_calib.h'daki her motorun 'center' degerine gider. Bu, robotun
 * notr/orta durusu. Webots HOME_ANGLES'in gercek karsiligi burasidir -
 * merkeze gidince durus mantikli gorunuyorsa kalibrasyon dogru demektir.
 *
 * GUVENLIK:
 *   - Motorlar SIRAYLA hareket eder (ayni anda degil) -> akim dusuk.
 *   - Hiz DUSUK -> yanlis giden motoru gorup gucu kesebilirsin.
 *   - Robot ASKIDA olmali.
 *
 * ST3020 (femur) : Serial1, hiz goal-pakette (0-4095 olcek)
 * AX-12A (coxa/tibia) : Serial2, ayri MOVING_SPEED komutu (0-1023 olcek)
 *
 * KOMUTLAR (Enter ile):
 *   h  - hepsini sirayla merkeze goturur
 *   t  - tum torklari kapat (serbest birak)
 *   <id>  - sadece o motoru merkeze goturur (orn: 5)
 */

#include <Arduino.h>
#include "motor_ids.h"
#include "motor_calib.h"

#define HEADER1 0xFF
#define HEADER2 0xFF
#define INST_WRITE 0x03

// --- ST3020 ---
#define STS_ADDR_TORQUE     40
#define STS_ADDR_GOAL_POS   42
#define STS_HIZ             200          // dusuk - gozle takip

// --- AX-12A ---
#define AX_ADDR_TORQUE      24
#define AX_ADDR_GOAL_POS    30
#define AX_ADDR_SPEED       32
#define AX_HIZ              100          // dusuk

// motor_ids.h: FEMUR_IDS ST3020. Bir id ST3020 mu?
bool isFemur(uint8_t id) {
  for (uint8_t i = 0; i < BACAK_SAYISI; i++)
    if (FEMUR_IDS[i] == id) return true;
  return false;
}

void sendPacket(HardwareSerial &ser, uint8_t id, uint8_t inst, uint8_t *p, uint8_t pLen) {
  uint8_t len = pLen + 2, cs = id + len + inst;
  ser.write(HEADER1); ser.write(HEADER2);
  ser.write(id); ser.write(len); ser.write(inst);
  for (uint8_t i = 0; i < pLen; i++) { ser.write(p[i]); cs += p[i]; }
  ser.write((uint8_t)(~cs));
  ser.flush();
}

void torque(uint8_t id, bool on) {
  bool f = isFemur(id);
  uint8_t addr = f ? STS_ADDR_TORQUE : AX_ADDR_TORQUE;
  uint8_t p[2] = {addr, (uint8_t)(on ? 1 : 0)};
  sendPacket(f ? FEMUR_SERIAL : AX_SERIAL, id, INST_WRITE, p, 2);
}

void gotoCenter(uint8_t id, uint16_t center) {
  if (isFemur(id)) {
    uint8_t p[7] = {STS_ADDR_GOAL_POS,
                    (uint8_t)(center & 0xFF), (uint8_t)(center >> 8),
                    0, 0,
                    (uint8_t)(STS_HIZ & 0xFF), (uint8_t)(STS_HIZ >> 8)};
    sendPacket(FEMUR_SERIAL, id, INST_WRITE, p, 7);
  } else {
    // AX: once hiz, sonra pozisyon
    uint8_t ps[3] = {AX_ADDR_SPEED, (uint8_t)(AX_HIZ & 0xFF), (uint8_t)(AX_HIZ >> 8)};
    sendPacket(AX_SERIAL, id, INST_WRITE, ps, 3);
    delay(10);
    uint8_t pp[3] = {AX_ADDR_GOAL_POS, (uint8_t)(center & 0xFF), (uint8_t)(center >> 8)};
    sendPacket(AX_SERIAL, id, INST_WRITE, pp, 3);
  }
}

void homeMotor(const MotorCalib &c) {
  torque(c.id, true);
  delay(20);
  gotoCenter(c.id, c.center);
  Serial.print(F("  ID ")); Serial.print(c.id);
  Serial.print(F(" -> merkez ")); Serial.print(c.center);
  Serial.print(F(" ("));
  Serial.print(isFemur(c.id) ? F("ST3020") : F("AX-12A"));
  if (c.wrap) Serial.print(F(", sarma"));
  Serial.println(F(")"));
}

void homeAll() {
  Serial.println(F("\n== 18 MOTOR SIRAYLA MERKEZE =="));
  Serial.println(F("(her motor arasi 600ms - gozle takip et)"));
  for (uint8_t i = 0; i < 18; i++) {
    homeMotor(MOTOR_CALIB[i]);
    delay(600);
  }
  Serial.println(F("Bitti. Durus dogru mu?"));
}

void torkKapatAll() {
  for (uint8_t i = 0; i < 18; i++) { torque(MOTOR_CALIB[i].id, false); delay(15); }
  Serial.println(F("Tum torklar KAPALI - motorlar serbest."));
}

void setup() {
  Serial.begin(115200);
  FEMUR_SERIAL.begin(FEMUR_BAUD);
  AX_SERIAL.begin(AX_BAUD);
  delay(500);

  Serial.println(F("==== HOME (merkeze git) ===="));
  Serial.println(F("Robot ASKIDA olmali!"));
  Serial.println(F("Komut: h=hepsi merkeze | t=tork kapat | <id>=tek motor"));
  Serial.println(F("\nOtomatik baslamiyor - once 'h' gonder."));
}

void loop() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf.trim();
      if (buf.length()) {
        if (buf == "h" || buf == "H") homeAll();
        else if (buf == "t" || buf == "T") torkKapatAll();
        else {
          uint8_t id = buf.toInt();
          const MotorCalib* c = calibBul(id);
          if (c) { Serial.print(F("Tek motor:")); homeMotor(*c); }
          else Serial.println(F("Gecersiz komut/ID."));
        }
      }
      buf = "";
    } else buf += c;
  }
}
