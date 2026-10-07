/*
 * AX-12A ID atama araci - Arduino Mega 2560
 *
 * !!! HATTA SADECE TEK MOTOR BAGLI OLMALI !!!
 * Bu kod "hatta kim varsa ID'sini degistir" mantigiyla calisir (broadcast).
 * Birden fazla motor bagliysa HEPSI ayni ID'yi alir ve durum kotulesir.
 *
 * KULLANIM:
 *   1. Asagidaki YENI_ID degerini istedigin degere ayarla (1, 2, 3, 4...)
 *   2. Hatta SADECE bir motor bagla
 *   3. Yukle
 *   4. Monitorde dogrulamayi izle
 *   5. O motoru cikar, sirakini bagla, YENI_ID'yi artir, tekrarla
 *
 * BAGLANTI: mega_ax12_test.cpp ile ayni
 *   Mega TX1 (pin 18) --[1k]--+--- AX-12A DATA
 *   Mega RX1 (pin 19) --------+
 *   Mega GND ----------------------- AX-12A GND ---- Guc (-)
 *   Guc (+) 11V -------------------- AX-12A VCC
 */

#include <Arduino.h>

// ================= BURAYI DEGISTIR =================
#define YENI_ID   1        // Bu motora atanacak ID
// ===================================================

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL

#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03
#define BROADCAST_ID   0xFE     // 254 = hattaki herkes

#define ADDR_ID        3        // AX-12A kontrol tablosunda ID adresi

void sendPacket(uint8_t id, uint8_t instruction, uint8_t *params, uint8_t paramLen)
{
  uint8_t length = paramLen + 2;
  uint8_t checksum = id + length + instruction;

  SERVO_SERIAL.write(HEADER1);
  SERVO_SERIAL.write(HEADER2);
  SERVO_SERIAL.write(id);
  SERVO_SERIAL.write(length);
  SERVO_SERIAL.write(instruction);

  for (uint8_t i = 0; i < paramLen; i++) {
    SERVO_SERIAL.write(params[i]);
    checksum += params[i];
  }

  SERVO_SERIAL.write((uint8_t)(~checksum));
  SERVO_SERIAL.flush();
}

// Hatta kac motor cevap veriyor, ID'leri ne?
uint8_t scanAndReport()
{
  uint8_t found = 0;

  for (uint16_t id = 0; id < 254; id++) {
    while (SERVO_SERIAL.available()) SERVO_SERIAL.read();

    sendPacket((uint8_t)id, INST_PING, NULL, 0);

    unsigned long start = millis();
    uint8_t bytesIn = 0;
    while (millis() - start < 20) {
      if (SERVO_SERIAL.available()) {
        SERVO_SERIAL.read();
        bytesIn++;
      }
    }

    if (bytesIn > 6) {                    // 6 byte = kendi echo'muz
      Serial.print(F("    ID: "));
      Serial.println(id);
      found++;
    }
  }
  return found;
}

void setup()
{
  Serial.begin(115200);
  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);

  Serial.println(F("\n=== AX-12A ID ATAMA ==="));
  Serial.print(F("Atanacak ID: "));
  Serial.println(YENI_ID);

  // --- Once mevcut durumu gor ---
  Serial.println(F("\n[1] Hattaki mevcut motorlar:"));
  uint8_t before = scanAndReport();

  if (before == 0) {
    Serial.println(F("    (yok)"));
    Serial.println(F("\nHATA: Hicbir motor bulunamadi. ID atanamaz."));
    Serial.println(F("Kontrol et: GND ortak mi? 11V var mi? DATA hatti dogru mu?"));
    return;
  }

  if (before > 1) {
    Serial.print(F("\n!!! DUR: Hatta "));
    Serial.print(before);
    Serial.println(F(" motor var !!!"));
    Serial.println(F("Bu kod broadcast ile yazar - HEPSI ayni ID'yi alir."));
    Serial.println(F("Sadece TEK motor birakip tekrar dene. ID YAZILMADI."));
    return;                               // Guvenlik: yazma
  }

  // --- Tek motor var, ID yaz ---
  Serial.println(F("\n[2] ID yaziliyor..."));
  uint8_t params[2] = { ADDR_ID, YENI_ID };
  sendPacket(BROADCAST_ID, INST_WRITE, params, 2);
  delay(200);                             // EEPROM yazmasi icin sure taniyoruz

  // --- Dogrula ---
  Serial.println(F("\n[3] Dogrulama - simdi hatta olanlar:"));
  uint8_t after = scanAndReport();

  if (after == 0) {
    Serial.println(F("    (yok)"));
    Serial.println(F("\nUYARI: Yazma sonrasi motor cevap vermiyor."));
  } else {
    Serial.println(F("\nBitti. Yukarida YENI_ID gorunuyorsa basarili."));
    Serial.println(F("Sonraki motor icin: bunu cikar, digerini bagla,"));
    Serial.println(F("YENI_ID'yi artir, tekrar yukle."));
  }
}

void loop()
{
  // Tek seferlik islem, loop bos
}
