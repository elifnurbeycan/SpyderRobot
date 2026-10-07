/*
 * AX-12A tek motor hareket testi - Arduino Mega 2560
 *
 * DIKKAT: AX-12A, ST3020'den FARKLI protokol kullanir (Dynamixel 1.0).
 *   - Goal Position adresi 30 (ST3020'de 42)
 *   - Torque Enable adresi 24 (ST3020'de 40)
 *   - Pozisyon araligi 0..1023 (ST3020'de 0..4095)
 *   - Baudrate 1 Mbps (ayni)
 *   - Checksum ayni: ~(ID + Length + Instruction + params)
 *
 * BAGLANTI:
 *   Mega TX1 (pin 18) --[1k]--+--- AX-12A DATA
 *   Mega RX1 (pin 19) --------+
 *   Mega GND ----------------------- AX-12A GND ---- Guc kaynagi (-)
 *   Guc kaynagi (+) 11V ------------ AX-12A VCC
 *
 * AX-12A calisma araligi 9-12V, onerilen 11.1V. Mevcut 11V uygun.
 *
 * !!! KONNEKTOR UYARISI !!!
 * AX-12A pin sirasi ST3020 ile AYNI DEGIL. Genelde GND / VCC / DATA.
 * ST3020 kablosunu koru koruna takma - VCC ile DATA yer degistirirse motor yanar.
 * Takmadan once multimetreyle GND-VCC arasinda 11V oldugunu dogrula.
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL   // AX-12A fabrika ayari
#define SERVO_ID     1           // Fabrika ayari. Tarama sonucu farkliysa degistir.

// Dynamikxel 1.0 protokol sabitleri
#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03

// AX-12A kontrol tablosu adresleri (ST3020'den FARKLI)
#define ADDR_TORQUE_ENABLE   24
#define ADDR_GOAL_POSITION   30
#define ADDR_MOVING_SPEED    32

// AX-12A pozisyon limitleri: 0..1023 => 0..300 derece
#define AX12_POS_MIN     0
#define AX12_POS_MAX     1023
#define AX12_POS_CENTER  512      // ~150 derece, orta nokta

// ---------------------------------------------------------------
// Paket gonderme. Dynamixel 1.0 checksum = ~(ID + Length + Instruction + params)
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
// Hiz ayari. 0 = maksimum hiz (limitsiz), 1..1023 = kademeli.
// ---------------------------------------------------------------
void setSpeed(uint8_t id, uint16_t speed)
{
  uint8_t params[3];
  params[0] = ADDR_MOVING_SPEED;
  params[1] = speed & 0xFF;
  params[2] = (speed >> 8) & 0xFF;
  sendPacket(id, INST_WRITE, params, 3);
}

// ---------------------------------------------------------------
// Pozisyon komutu. AX-12A'da pozisyon 0..1023 (0..300 derece).
// Aralik disi deger gonderilirse motor komutu reddeder.
// ---------------------------------------------------------------
void setPosition(uint8_t id, uint16_t position)
{
  if (position > AX12_POS_MAX) position = AX12_POS_MAX;   // guvenlik siniri

  uint8_t params[3];
  params[0] = ADDR_GOAL_POSITION;
  params[1] = position & 0xFF;            // pozisyon dusuk byte
  params[2] = (position >> 8) & 0xFF;     // pozisyon yuksek byte
  sendPacket(id, INST_WRITE, params, 3);
}

// ---------------------------------------------------------------
// ID taramasi. Baudrate ve checksum ST3020 ile ayni oldugu icin
// bu tarama her iki motor tipini de bulur.
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
    Serial.println(F("  Kontrol et: GND ortak mi? 11V var mi? DATA hatti dogru mu?"));
  }
  Serial.println(F("--- Tarama bitti ---"));
}

void setup()
{
  Serial.begin(115200);                   // USB monitor
  SERVO_SERIAL.begin(SERVO_BAUD);         // Motor hatti
  delay(500);

  Serial.println(F("\nAX-12A test basliyor"));

  scanForServos();                        // Once kim var bakalim

  torqueEnable(SERVO_ID, true);
  delay(100);
  setSpeed(SERVO_ID, 200);                // Yavas hareket, gozle takip edilebilsin
  delay(100);
  Serial.println(F("Tork acildi. Hareket dongusu basliyor.\n"));
}

void loop()
{
  // 512 = orta nokta (~150 derece)
  Serial.println(F("-> 512 (orta)"));
  setPosition(SERVO_ID, AX12_POS_CENTER);
  delay(2000);

  // 812 = orta + ~88 derece. 1023 sinirinin altinda kaliyor.
  Serial.println(F("-> 812 (+88 derece)"));
  setPosition(SERVO_ID, 812);
  delay(2000);
}
