/*
 * mega_scan_all.cpp - ST3020 + AX-12A toplu ID tarama
 *
 * ST3020 : Serial1 (TX1=18, RX1=19) @ 1 Mbaud
 * AX-12A : Serial2 (TX2=16, RX2=17) @ 1 Mbaud
 * ID araligi: 1-18
 *
 * Komutlar:
 *   s  - tekrar tara
 */

#include <Arduino.h>

#define ST_SERIAL   Serial1     // ST3020
#define AX_SERIAL   Serial2     // AX-12A
#define BAUD        1000000UL

// PING paketi: FF FF id 02 01 ~(id+3)
void sendPing(HardwareSerial &ser, uint8_t id)
{
  uint8_t cs = (uint8_t)(~(id + 0x02 + 0x01));
  ser.write(0xFF); ser.write(0xFF);
  ser.write(id);   ser.write(0x02);
  ser.write(0x01); ser.write(cs);
  ser.flush();
}

// TX echo = 6 byte, motor yaniti = 6 byte → toplam > 6 ise cevap var
bool ping(HardwareSerial &ser, uint8_t id)
{
  while (ser.available()) ser.read();
  sendPing(ser, id);
  unsigned long t = millis();
  uint8_t n = 0;
  while (millis() - t < 20) {
    if (ser.available()) { ser.read(); n++; }
  }
  return (n > 6);
}

void tara(HardwareSerial &ser, const char *tip)
{
  Serial.print(F("\n[ ")); Serial.print(tip); Serial.println(F(" ] taranıyor (ID 1-18)..."));
  uint8_t bulunan = 0;
  for (uint8_t id = 1; id <= 18; id++) {
    if (ping(ser, id)) {
      Serial.print(F("  BULUNDU -> ID: ")); Serial.println(id);
      bulunan++;
    }
  }
  if (bulunan == 0) Serial.println(F("  (hicbir motor cevap vermedi)"));
  else { Serial.print(F("  Toplam: ")); Serial.println(bulunan); }
}

void setup()
{
  Serial.begin(115200);
  ST_SERIAL.begin(BAUD);
  AX_SERIAL.begin(BAUD);
  delay(500);

  Serial.println(F("==== ST3020 + AX-12A ID TARAMA ===="));
  Serial.println(F("ST3020 : Serial1 (TX=18, RX=19) [1Mbaud]"));
  Serial.println(F("AX-12A : Serial2 (TX=16, RX=17) [1Mbaud]"));

  tara(ST_SERIAL, "ST3020");
  tara(AX_SERIAL, "AX-12A");

  Serial.println(F("\n's' ile tekrar tara."));
}

void loop()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 's' || c == 'S') {
    tara(ST_SERIAL, "ST3020");
    tara(AX_SERIAL, "AX-12A");
    Serial.println(F("\nBitti."));
  }
}
