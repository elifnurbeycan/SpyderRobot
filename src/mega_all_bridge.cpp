/*
 * mega_all_bridge.cpp - 18 motor web arayuzu seri koprusu (ST3020 + AX-12A)
 *
 * FEMUR -> ST3020 (Feetech STS) : Serial1 (TX1=18, RX1=19), 0-4095, tip 'S'
 * COXA/TIBIA -> AX-12A (Dynamixel): Serial2 (TX2=16, RX2=17), 0-1023, tip 'A'
 *
 * Komutlar (PC'den, \n ile biter):
 *   A              - Tum motorlarin pozisyonunu oku
 *   M <id> <pos>   - Motoru pozisyona goturur
 *   T <id> <0|1>   - Tork ac/kapat
 *   UNLIM <id>     - O motorun aci limitini tam aralik yapar
 *   UNLIMALL       - Tum motorlarin limitini acar
 *
 * Yanitlar:
 *   P <id> <pos>   - Pozisyon (-1 = hata)
 *   A DONE / OK / READY
 *
 * Motor tipi ID'den otomatik cozulur (asagidaki tabloya gore).
 */

#include <Arduino.h>

#define ST_SERIAL   Serial1
#define AX_SERIAL   Serial2
#define BAUD        1000000UL

// --- ST3020 (Feetech STS) kontrol tablosu ---
#define STS_ADDR_LOCK        55
#define STS_ADDR_MIN_ANGLE    9   // CW/CCW angle limit (STS: 9-10 min, 11-12 max)
#define STS_ADDR_MAX_ANGLE   11
#define STS_ADDR_TORQUE      40
#define STS_ADDR_GOAL_POS    42
#define STS_ADDR_PRESENT_POS 56
#define STS_POS_MAX          4095

// --- AX-12A (Dynamixel 1.0) kontrol tablosu ---
#define AX_ADDR_CW_LIMIT      6   // 6-7 CW, 8-9 CCW angle limit
#define AX_ADDR_CCW_LIMIT     8
#define AX_ADDR_TORQUE       24
#define AX_ADDR_GOAL_POS     30
#define AX_ADDR_MOVING_SPEED 32
#define AX_ADDR_PRESENT_POS  36
#define AX_POS_MAX           1023

#define INST_READ   0x02
#define INST_WRITE  0x03

// --- Motor tablosu: ID -> tip ('S'=ST3020, 'A'=AX-12A) ---
struct Motor { uint8_t id; char tip; };
const Motor MOTORS[] = {
  // Femurlar (ST3020)
  {2,'S'},{5,'S'},{8,'S'},{11,'S'},{14,'S'},{17,'S'},
  // Coxa + Tibia (AX-12A)
  {1,'A'},{3,'A'},{4,'A'},{6,'A'},{7,'A'},{9,'A'},
  {10,'A'},{12,'A'},{13,'A'},{15,'A'},{16,'A'},{18,'A'}
};
const uint8_t MOTOR_COUNT = sizeof(MOTORS) / sizeof(MOTORS[0]);

char tipBul(uint8_t id) {
  for (uint8_t i = 0; i < MOTOR_COUNT; i++)
    if (MOTORS[i].id == id) return MOTORS[i].tip;
  return 0;
}

HardwareSerial& hatBul(char tip) {
  return (tip == 'S') ? ST_SERIAL : AX_SERIAL;
}

// --- Dynamixel/STS ortak paket (ikisi de ayni checksum formati) ---
void sendPacket(HardwareSerial &ser, uint8_t id, uint8_t inst, uint8_t *p, uint8_t pLen)
{
  uint8_t len = pLen + 2;
  uint8_t cs  = id + len + inst;
  ser.write(0xFF); ser.write(0xFF);
  ser.write(id);   ser.write(len);
  ser.write(inst);
  for (uint8_t i = 0; i < pLen; i++) { ser.write(p[i]); cs += p[i]; }
  ser.write((uint8_t)(~cs));
  ser.flush();
}

void setPos(uint8_t id, uint16_t pos)
{
  char tip = tipBul(id);
  if (!tip) return;
  HardwareSerial &ser = hatBul(tip);

  if (tip == 'S') {
    if (pos > STS_POS_MAX) pos = STS_POS_MAX;
    uint8_t p[7] = {STS_ADDR_GOAL_POS,
                    (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
                    0, 0, 0x2C, 0x01};        // hiz ~300
    sendPacket(ser, id, INST_WRITE, p, 7);
  } else {
    if (pos > AX_POS_MAX) pos = AX_POS_MAX;
    uint8_t p[3] = {AX_ADDR_GOAL_POS,
                    (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8)};
    sendPacket(ser, id, INST_WRITE, p, 3);
  }
}

void torque(uint8_t id, bool on)
{
  char tip = tipBul(id);
  if (!tip) return;
  uint8_t addr = (tip == 'S') ? STS_ADDR_TORQUE : AX_ADDR_TORQUE;
  uint8_t p[2] = {addr, (uint8_t)(on ? 1 : 0)};
  sendPacket(hatBul(tip), id, INST_WRITE, p, 2);
}

// Present position oku. STS ve AX ayni READ formati, farkli adres.
int16_t readPos(uint8_t id)
{
  char tip = tipBul(id);
  if (!tip) return -1;
  HardwareSerial &ser = hatBul(tip);
  uint8_t addr = (tip == 'S') ? STS_ADDR_PRESENT_POS : AX_ADDR_PRESENT_POS;

  while (ser.available()) ser.read();
  uint8_t p[2] = {addr, 2};
  sendPacket(ser, id, INST_READ, p, 2);

  // TX echo 8 byte + yanit 8 byte = 16. Yanit govdesi: FF FF id len err d0 d1 cs
  // d0 = buf[13], d1 = buf[14] (echo 8 + header 5 sonrasi)
  uint8_t buf[16]; uint8_t idx = 0;
  unsigned long t = millis();
  while (idx < 16 && millis() - t < 50)
    if (ser.available()) buf[idx++] = ser.read();
  if (idx < 16) return -1;
  return (int16_t)((uint16_t)buf[13] | ((uint16_t)buf[14] << 8));
}

void eepromLock(uint8_t id, bool lock)   // sadece STS
{
  uint8_t p[2] = {STS_ADDR_LOCK, (uint8_t)(lock ? 1 : 0)};
  sendPacket(ST_SERIAL, id, INST_WRITE, p, 2);
  delay(50);
}

// Aci limitini tam aralik yap (motoru serbest birakir)
void unlim(uint8_t id)
{
  char tip = tipBul(id);
  if (tip == 'S') {
    eepromLock(id, false); delay(20);
    // STS min=0 (addr 9-10), max=4095 (addr 11-12), ardisik yaz
    uint8_t p[5] = {STS_ADDR_MIN_ANGLE, 0x00, 0x00, 0xFF, 0x0F};
    sendPacket(ST_SERIAL, id, INST_WRITE, p, 5);
    delay(100);
    eepromLock(id, true);
  } else if (tip == 'A') {
    // AX CW=0 (addr 6-7), CCW=1023 (addr 8-9), ardisik yaz
    uint8_t p[5] = {AX_ADDR_CW_LIMIT, 0x00, 0x00, 0xFF, 0x03};
    sendPacket(AX_SERIAL, id, INST_WRITE, p, 5);
    delay(100);
  }
}

void setup()
{
  Serial.begin(115200);
  ST_SERIAL.begin(BAUD);
  AX_SERIAL.begin(BAUD);
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
      int16_t pos = readPos(MOTORS[i].id);
      Serial.print(F("P ")); Serial.print(MOTORS[i].id);
      Serial.print(' ');     Serial.println(pos);
    }
    Serial.println(F("A DONE"));
  }
  else if (verb == "M") {
    String rest = cmd.substring(s1 + 1);
    int s2 = rest.indexOf(' ');
    uint8_t  id  = rest.substring(0, s2).toInt();
    uint16_t pos = rest.substring(s2 + 1).toInt();
    setPos(id, pos);
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
    uint8_t id = cmd.substring(s1 + 1).toInt();
    unlim(id);
    Serial.print(F("OK UNLIM ")); Serial.println(id);
  }
  else if (verb == "UNLIMALL") {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      unlim(MOTORS[i].id);
      Serial.print(F("OK UNLIM ")); Serial.println(MOTORS[i].id);
    }
    Serial.println(F("UNLIMALL DONE"));
  }
}
