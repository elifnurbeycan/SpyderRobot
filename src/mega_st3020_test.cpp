/*
 * ST3020 (Feetech STS serisi) tek motor hareket testi - Arduino Mega 2560
 *
 * BAGLANTI:
 *   Mega TX1 (pin 18) --[1k]--+--- ST3020 DATA
 *   Mega RX1 (pin 19) --------+
 *   Mega GND ----------------------- ST3020 GND ---- Guc kaynagi (-)
 *   Guc kaynagi (+) 7.4V ----------- ST3020 VCC
 *
 * Motoru Arduino'nun 5V pininden BESLEME. Ayri kaynak kullan.
 *
 * Bu kod motoru surekli iki pozisyon arasinda gidip getirir.
 * Gozle gorulur hareket = baglanti dogru demektir.
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL   // ST3020 fabrika ayari
#define SERVO_ID     1           // Fabrika ayari. Bilmiyorsan asagidaki taramayi kullan.

// STS/SCS protokol sabitleri
#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03

// STS bellek adresleri
#define ADDR_TORQUE_ENABLE   40
#define ADDR_GOAL_POSITION   42

// ---------------------------------------------------------------
// Paket gonderme. STS checksum = ~(ID + Length + Instruction + params)
// ---------------------------------------------------------------
void sendPacket(uint8_t id, uint8_t instruction, uint8_t *params, uint8_t paramLen)
{
  uint8_t length = paramLen + 2;          // params + instruction + checksum
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
  SERVO_SERIAL.flush();                   // Tum byte'lar cikana kadar bekle
}

// ---------------------------------------------------------------
// Torku ac. Tork kapaliyken motor komut alsa da donmez.
// ---------------------------------------------------------------
void torqueEnable(uint8_t id, bool on)
{
  uint8_t params[2] = { ADDR_TORQUE_ENABLE, (uint8_t)(on ? 1 : 0) };
  sendPacket(id, INST_WRITE, params, 2);
}

// ---------------------------------------------------------------
// Pozisyon komutu. STS'de pozisyon 0..4095 arasi (0..360 derece).
// Hiz da ayni yazmada gonderiliyor (adres 46-47).
// ---------------------------------------------------------------
void setPosition(uint8_t id, uint16_t position, uint16_t speed)
{
  uint8_t params[7];
  params[0] = ADDR_GOAL_POSITION;
  params[1] = position & 0xFF;            // pozisyon dusuk byte
  params[2] = (position >> 8) & 0xFF;     // pozisyon yuksek byte
  params[3] = 0;                          // adres 44-45: time, kullanmiyoruz
  params[4] = 0;
  params[5] = speed & 0xFF;               // hiz dusuk byte
  params[6] = (speed >> 8) & 0xFF;        // hiz yuksek byte
  sendPacket(id, INST_WRITE, params, 7);
}

// ---------------------------------------------------------------
// ID taramasi. Motorun ID'sini bilmiyorsan bunu kullan.
// Cevap veren ID'yi USB monitorde yazar.
// ---------------------------------------------------------------
void scanForServos()
{
  Serial.println(F("--- ID taramasi basliyor (0-253) ---"));
  uint8_t found = 0;

  for (uint16_t id = 0; id < 254; id++) {
    while (SERVO_SERIAL.available()) SERVO_SERIAL.read();   // tamponu temizle

    sendPacket((uint8_t)id, INST_PING, NULL, 0);

    unsigned long start = millis();
    uint8_t bytesIn = 0;
    while (millis() - start < 20) {
      if (SERVO_SERIAL.available()) {
        SERVO_SERIAL.read();
        bytesIn++;
      }
    }

    // Kendi gonderdigimiz byte'lar da hatta geri okunur (half-duplex echo).
    // Gonderilen ping 6 byte. Bundan fazlasi gercek cevap demektir.
    if (bytesIn > 6) {
      Serial.print(F("  Motor bulundu -> ID: "));
      Serial.println(id);
      found++;
    }
  }

  if (found == 0) {
    Serial.println(F("  Hicbir motor cevap vermedi."));
    Serial.println(F("  Kontrol et: GND ortak mi? 7.4V var mi? DATA hatti dogru mu?"));
  }
  Serial.println(F("--- Tarama bitti ---"));
}

void setup()
{
  Serial.begin(115200);                   // USB monitor
  SERVO_SERIAL.begin(SERVO_BAUD);         // Motor hatti
  delay(500);

  Serial.println(F("\nST3020 test basliyor"));

  scanForServos();                        // Once kim var bakalim

  torqueEnable(SERVO_ID, true);
  delay(100);
  Serial.println(F("Tork acildi. Hareket dongusu basliyor.\n"));
}

void loop()
{
  // 2048 = orta nokta (180 derece)
  Serial.println(F("-> 2048 (orta)"));
  setPosition(SERVO_ID, 2048, 1000);
  delay(2000);

  // 3072 = orta + 90 derece
  Serial.println(F("-> 3072 (+90 derece)"));
  setPosition(SERVO_ID, 3072, 1000);
  delay(2000);
}
