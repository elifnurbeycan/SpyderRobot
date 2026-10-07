/*
 * mega_home_read.cpp - Tum motorlarin mevcut pozisyonunu oku ve yazdir
 *
 * ST3020 : Serial1 (TX=18, RX=19) | IDs: 2,5,8,11,14,17 | addr 56 | 0-4095
 * AX-12A : Serial2 (TX=16, RX=17) | IDs: 1,3,4,6,7,9,10,12,13,15,16,18 | addr 36 | 0-1023
 *
 * Komutlar: 'r' ile tekrar oku
 */

#include <Arduino.h>

#define ST_SERIAL   Serial1
#define AX_SERIAL   Serial2
#define BAUD        1000000UL

#define INST_READ   0x02

// ST3020: addr 56, AX-12A: addr 36
const uint8_t ST_IDS[]  = {2, 5, 8, 11, 14, 17};
const uint8_t AX_IDS[]  = {1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18};
const uint8_t ST_COUNT  = 6;
const uint8_t AX_COUNT  = 12;

void sendReadPkt(HardwareSerial &ser, uint8_t id, uint8_t addr)
{
  uint8_t len = 4;  // paramLen=2 + 2
  uint8_t cs  = (uint8_t)(~(id + len + INST_READ + addr + 2));
  ser.write(0xFF); ser.write(0xFF);
  ser.write(id);   ser.write(len);
  ser.write(INST_READ);
  ser.write(addr); ser.write(0x02);
  ser.write(cs);
  ser.flush();
}

// Half-duplex: TX echo 8 byte + motor yaniti 8 byte = 16 toplam
// pos: buf[13] | (buf[14] << 8)
int16_t readPos(HardwareSerial &ser, uint8_t id, uint8_t addr)
{
  while (ser.available()) ser.read();
  sendReadPkt(ser, id, addr);

  uint8_t buf[16]; uint8_t idx = 0;
  unsigned long t = millis();
  while (idx < 16 && millis() - t < 50)
    if (ser.available()) buf[idx++] = ser.read();

  if (idx < 16) return -1;
  return (int16_t)((uint16_t)buf[13] | ((uint16_t)buf[14] << 8));
}

void readAll()
{
  Serial.println(F("\n# --- HOME POZISYONU ---"));

  // ST3020
  Serial.println(F("\nST3020_HOME = {  # femur motorlari (Serial1, 0-4095)"));
  for (uint8_t i = 0; i < ST_COUNT; i++) {
    int16_t pos = readPos(ST_SERIAL, ST_IDS[i], 56);
    Serial.print(F("    ")); Serial.print(ST_IDS[i]);
    Serial.print(F(": "));   Serial.print(pos);
    Serial.print(F(",   # ID ")); Serial.println(ST_IDS[i]);
  }
  Serial.println(F("}"));

  // AX-12A
  Serial.println(F("\nAX12_HOME = {  # coxa+tibia motorlari (Serial2, 0-1023)"));
  for (uint8_t i = 0; i < AX_COUNT; i++) {
    int16_t pos = readPos(AX_SERIAL, AX_IDS[i], 36);
    Serial.print(F("    ")); Serial.print(AX_IDS[i]);
    Serial.print(F(": "));   Serial.print(pos);
    Serial.print(F(",   # ID ")); Serial.println(AX_IDS[i]);
  }
  Serial.println(F("}"));

  Serial.println(F("\n# --- BITTI ---"));
}

void setup()
{
  Serial.begin(115200);
  ST_SERIAL.begin(BAUD);
  AX_SERIAL.begin(BAUD);
  delay(500);

  Serial.println(F("==== HOME POZISYON OKUYUCU ===="));
  Serial.println(F("Tum motorlar okunuyor..."));
  readAll();
  Serial.println(F("\n'r' ile tekrar oku."));
}

void loop()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'r' || c == 'R') readAll();
}
