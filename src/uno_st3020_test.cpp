/*
 * ST3020 tek motor hareket testi - Arduino UNO
 *
 * !!! DIKKAT !!!
 * UNO'da tek donanim UART var ve o da USB'ye bagli.
 * Motor o hatti kullandigi icin YUKLEME SIRASINDA MOTOR DATA KABLOSU CIKIK OLMALI.
 *
 * SIRA:
 *   1. Motorun DATA kablosunu Uno'dan CIKAR
 *   2. Kodu yukle
 *   3. DATA kablosunu TAK
 *   4. Uno'ya RESET at
 *   5. Motor donmeli
 *
 * BAGLANTI:
 *   Uno TX (pin 1) --[1k]--+--- ST3020 DATA
 *   Uno RX (pin 0) --------+
 *   Uno GND ----------------------- ST3020 GND ---- Guc kaynagi (-)
 *   Guc kaynagi (+) 7.4V ----------- ST3020 VCC
 *
 * Motoru Arduino'nun 5V pininden BESLEME. Ayri kaynak kullan.
 *
 * DEBUG: Serial kullanilamadigi icin 13 nolu pindeki dahili LED kullaniliyor.
 *   Acilista 3 kez hizli yanip sonme = kod calisiyor
 *   Sonrasinda her komutta 1 kez yanip sonme = komut gonderildi
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial
#define SERVO_BAUD   1000000UL   // ST3020 fabrika ayari. Olmazsa 500000UL dene.
#define SERVO_ID     1           // Fabrika ayari. Uno'da tarama yapilamaz.
#define LED_PIN      13

// STS/SCS protokol sabitleri
#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_WRITE     0x03

// STS bellek adresleri
#define ADDR_TORQUE_ENABLE   40
#define ADDR_GOAL_POSITION   42

// ---------------------------------------------------------------
// LED ile isaret ver. Serial debug yok, elimizde bu var.
// ---------------------------------------------------------------
void blink(uint8_t times, uint16_t onMs)
{
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(onMs);
    digitalWrite(LED_PIN, LOW);
    delay(onMs);
  }
}

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

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  // Acilista 3 hizli flash: kod calisiyor demek.
  // Bu flash'i gormuyorsan kod yuklenmemis ya da Uno beslenmiyor.
  blink(3, 100);

  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);

  torqueEnable(SERVO_ID, true);
  delay(100);
}

void loop()
{
  // 2048 = orta nokta (180 derece)
  setPosition(SERVO_ID, 2048, 1000);
  blink(1, 50);
  delay(2000);

  // 3072 = orta + 90 derece
  setPosition(SERVO_ID, 3072, 1000);
  blink(1, 50);
  delay(2000);
}
