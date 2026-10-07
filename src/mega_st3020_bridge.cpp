/*
 * mega_st3020_bridge.cpp - Web arayüzü için seri köprü
 *
 * Komutlar (PC'den, \n ile biter):
 *   A              - Tum motorlarin pozisyonunu oku
 *   M <id> <pos>   - Motoru pozisyona götür
 *   T <id> <0|1>   - Tork aç/kapat
 *
 * Yanitlar:
 *   P <id> <pos>   - Pozisyon (-1 = hata)
 *   A DONE
 *   OK
 *   READY
 *
 * BAGLANTI: Mega TX1(18) --[1k]--+-- ST3020 DATA
 *           Mega RX1(19) --------+
 */

#include <Arduino.h>

#define SERVO_SERIAL  Serial1
#define SERVO_BAUD    1000000UL

#define INST_READ   0x02
#define INST_WRITE  0x03

#define ADDR_LOCK           55   // 0=açık, 1=kilitli
#define ADDR_MIN_ANGLE       6   // 2 byte, little-endian
#define ADDR_MAX_ANGLE       8   // 2 byte, little-endian
#define ADDR_TORQUE_ENABLE  40
#define ADDR_GOAL_POSITION  42
#define ADDR_PRESENT_POS    56

const uint8_t MOTOR_IDS[] = {2, 5, 8, 11, 14, 17};
const uint8_t MOTOR_COUNT = 6;

void sendPacket(uint8_t id, uint8_t inst, uint8_t *p, uint8_t pLen)
{
  uint8_t len = pLen + 2;
  uint8_t cs  = id + len + inst;
  SERVO_SERIAL.write(0xFF); SERVO_SERIAL.write(0xFF);
  SERVO_SERIAL.write(id);   SERVO_SERIAL.write(len);
  SERVO_SERIAL.write(inst);
  for (uint8_t i = 0; i < pLen; i++) { SERVO_SERIAL.write(p[i]); cs += p[i]; }
  SERVO_SERIAL.write((uint8_t)(~cs));
  SERVO_SERIAL.flush();
}

void setPos(uint8_t id, uint16_t pos, uint16_t spd)
{
  if (pos > 4095) pos = 4095;
  uint8_t p[7] = {ADDR_GOAL_POSITION,
                  (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
                  0, 0,
                  (uint8_t)(spd & 0xFF), (uint8_t)(spd >> 8)};
  sendPacket(id, INST_WRITE, p, 7);
}

void eepromLock(uint8_t id, bool lock)
{
  uint8_t p[2] = {ADDR_LOCK, (uint8_t)(lock ? 1 : 0)};
  sendPacket(id, INST_WRITE, p, 2);
  delay(50);
}

void setAngleLimits(uint8_t id, uint16_t minA, uint16_t maxA)
{
  // EEPROM kilidinii ac
  eepromLock(id, false);
  delay(20);

  // Min limit yaz (addr 6, 2 byte)
  uint8_t p1[4] = {ADDR_MIN_ANGLE,
                   (uint8_t)(minA & 0xFF), (uint8_t)(minA >> 8),
                   (uint8_t)(maxA & 0xFF)};
  // ST3020'de adresler ardisik, tek pakette yazilabilir:
  // addr 6 = minL, 7 = minH, 8 = maxL, 9 = maxH
  uint8_t p2[5] = {ADDR_MIN_ANGLE,
                   (uint8_t)(minA & 0xFF), (uint8_t)(minA >> 8),
                   (uint8_t)(maxA & 0xFF), (uint8_t)(maxA >> 8)};
  sendPacket(id, INST_WRITE, p2, 5);
  delay(100);

  // Kilitle
  eepromLock(id, true);
}

void torque(uint8_t id, bool on)
{
  uint8_t p[2] = {ADDR_TORQUE_ENABLE, (uint8_t)(on ? 1 : 0)};
  sendPacket(id, INST_WRITE, p, 2);
}

int16_t readPos(uint8_t id)
{
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
  uint8_t p[2] = {ADDR_PRESENT_POS, 2};
  sendPacket(id, INST_READ, p, 2);

  // TX echo: 8 byte, motor yaniti: 8 byte → 16 toplam
  uint8_t buf[16]; uint8_t idx = 0;
  unsigned long t = millis();
  while (idx < 16 && millis() - t < 50)
    if (SERVO_SERIAL.available()) buf[idx++] = SERVO_SERIAL.read();
  if (idx < 16) return -1;
  return (int16_t)((uint16_t)buf[13] | ((uint16_t)buf[14] << 8));
}

void setup()
{
  Serial.begin(115200);
  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);
  Serial.println(F("READY"));
}

void loop()
{
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (!cmd.length()) return;

  int s1 = cmd.indexOf(' ');
  String verb = (s1 < 0) ? cmd : cmd.substring(0, s1);
  verb.toUpperCase();

  if (verb == "A") {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      int16_t pos = readPos(MOTOR_IDS[i]);
      Serial.print(F("P ")); Serial.print(MOTOR_IDS[i]);
      Serial.print(' ');     Serial.println(pos);
    }
    Serial.println(F("A DONE"));
  }
  else if (verb == "M") {
    String rest = cmd.substring(s1 + 1);
    int s2 = rest.indexOf(' ');
    uint8_t  id  = rest.substring(0, s2).toInt();
    uint16_t pos = rest.substring(s2 + 1).toInt();
    setPos(id, pos, 300);
    Serial.println(F("OK"));
  }
  else if (verb == "T") {
    String rest = cmd.substring(s1 + 1);
    int s2 = rest.indexOf(' ');
    uint8_t id = rest.substring(0, s2).toInt();
    uint8_t on = rest.substring(s2 + 1).toInt();
    torque(id, on);
    Serial.println(F("OK"));
  }
  else if (verb == "UNLIM") {
    // UNLIM <id>  → EEPROM açı limitini 0-4095 yap
    uint8_t id = cmd.substring(s1 + 1).toInt();
    setAngleLimits(id, 0, 4095);
    Serial.print(F("OK UNLIM ")); Serial.println(id);
  }
  else if (verb == "UNLIMALL") {
    // UNLIMALL → Tüm motorların limitini aç
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      setAngleLimits(MOTOR_IDS[i], 0, 4095);
      Serial.print(F("OK UNLIM ")); Serial.println(MOTOR_IDS[i]);
    }
    Serial.println(F("UNLIMALL DONE"));
  }
}
