/*
 * 6 adet ST3020 (femur) sirayla hareket testi - Arduino Mega 2560
 *
 * ID'ler: 2, 5, 8, 11, 14, 17
 *
 * Her motoru SIRAYLA merkez -> +10 derece -> merkez -> -10 derece -> merkez
 * seklinde oynatir. Boylece hangisinin hangi bacakta oldugunu gozle
 * dogrulayabilirsin.
 *
 * BAGLANTI:
 *   Mega TX1 (pin 18) --[2.2k]--+--- ST3020 DATA zinciri
 *   Mega RX1 (pin 19) ----------+
 *   Mega GND ------------------------ ST3020 GND ---- Guc (-)
 *   Guc (+) 11V --------------------- ST3020 VCC
 *
 * !!! GERILIM UYARISI !!!
 * ST3020 calisma araligi 6-14V. 5V YETERSIZDIR - motor calismaz.
 * Besleme 11-12V olmali.
 *
 * !!! GUC UYARISI !!!
 * 6 ST3020 ayni anda hareket ederse 6-10A cekebilir.
 * Bu kod SIRAYLA oynatiyor - tek seferde bir motor, akim dusuk kalir.
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL

#define MOTOR_SAYISI 6
const uint8_t FEMUR_IDS[MOTOR_SAYISI] = { 2, 5, 8, 11, 14, 17 };

// Feetech STS protokol sabitleri
#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03

// ST3020 kontrol tablosu adresleri
#define ADDR_TORQUE_ENABLE   40
#define ADDR_GOAL_POSITION   42

// ST3020: 0..4095 => 0..360 derece. Yani 1 derece = 4096/360 = 11.378 adim
#define STS_POS_MAX      4095
#define STS_POS_CENTER   2048
#define ADIM_PER_DERECE  11.378f

// Test acisi - kucuk tut, montajli sistemde carpma olmasin
#define TEST_ACI_DERECE  10

// Hareket hizi. Dusuk = yavas = gozle takip edilebilir
#define TEST_HIZ         500

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

bool pingServo(uint8_t id)
{
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();

  sendPacket(id, INST_PING, NULL, 0);

  unsigned long start = millis();
  uint8_t bytesIn = 0;
  while (millis() - start < 20) {
    if (SERVO_SERIAL.available()) {
      SERVO_SERIAL.read();
      bytesIn++;
    }
  }

  return (bytesIn > 6);        // 6 byte = kendi echo'muz
}

void torqueEnable(uint8_t id, bool on)
{
  uint8_t params[2] = { ADDR_TORQUE_ENABLE, (uint8_t)(on ? 1 : 0) };
  sendPacket(id, INST_WRITE, params, 2);
}

void setPosition(uint8_t id, uint16_t position, uint16_t speed)
{
  if (position > STS_POS_MAX) position = STS_POS_MAX;   // guvenlik siniri

  uint8_t params[7];
  params[0] = ADDR_GOAL_POSITION;
  params[1] = position & 0xFF;
  params[2] = (position >> 8) & 0xFF;
  params[3] = 0;                          // adres 44-45: time
  params[4] = 0;
  params[5] = speed & 0xFF;
  params[6] = (speed >> 8) & 0xFF;
  sendPacket(id, INST_WRITE, params, 7);
}

// Merkeze gore derece cinsinden pozisyon ver
void setAci(uint8_t id, float derece, uint16_t speed)
{
  long hedef = (long)STS_POS_CENTER + (long)(derece * ADIM_PER_DERECE);

  if (hedef < 0) hedef = 0;                       // guvenlik siniri
  if (hedef > STS_POS_MAX) hedef = STS_POS_MAX;

  setPosition(id, (uint16_t)hedef, speed);
}

// Beklenen ID'leri tara, eksikleri bildir
void taraVeDogrula()
{
  Serial.println(F("--- Femur taramasi ---"));
  uint8_t bulunan = 0;

  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    uint8_t id = FEMUR_IDS[i];
    if (pingServo(id)) {
      Serial.print(F("  OK    -> ID "));
      Serial.println(id);
      bulunan++;
    } else {
      Serial.print(F("  EKSIK -> ID "));
      Serial.print(id);
      Serial.println(F(" cevap vermedi"));
    }
  }

  Serial.print(F("Bulunan: "));
  Serial.print(bulunan);
  Serial.print(F(" / "));
  Serial.println(MOTOR_SAYISI);

  if (bulunan == 0) {
    Serial.println(F("\n  Hicbiri cevap vermiyor. Kontrol et:"));
    Serial.println(F("  - Besleme 11V mi? (5V YETERSIZ, motor calismaz)"));
    Serial.println(F("  - GND ortak mi?"));
    Serial.println(F("  - DATA hatti pin 18/19'a 2.2k ile bagli mi?"));
  }
  Serial.println(F("--- Tarama bitti ---\n"));
}

void setup()
{
  Serial.begin(115200);
  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);

  Serial.println(F("\n=== ST3020 FEMUR TESTI ==="));
  Serial.print(F("Test acisi: +/- "));
  Serial.print(TEST_ACI_DERECE);
  Serial.println(F(" derece"));
  Serial.println();

  taraVeDogrula();

  // Tum femurlarin torkunu ac ve merkeze al
  Serial.println(F("Tork aciliyor, hepsi merkeze aliniyor..."));
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    torqueEnable(FEMUR_IDS[i], true);
    delay(20);
    setAci(FEMUR_IDS[i], 0, TEST_HIZ);      // merkez
    delay(20);
  }
  delay(1500);                              // merkeze varmalarini bekle

  Serial.println(F("Sirayla test basliyor.\n"));
}

void loop()
{
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    uint8_t id = FEMUR_IDS[i];

    Serial.print(F("--- Femur ID "));
    Serial.print(id);
    Serial.println(F(" ---"));

    Serial.println(F("   +10 derece"));
    setAci(id, +TEST_ACI_DERECE, TEST_HIZ);
    delay(800);

    Serial.println(F("   merkez"));
    setAci(id, 0, TEST_HIZ);
    delay(800);

    Serial.println(F("   -10 derece"));
    setAci(id, -TEST_ACI_DERECE, TEST_HIZ);
    delay(800);

    Serial.println(F("   merkez"));
    setAci(id, 0, TEST_HIZ);
    delay(1200);                            // sonraki motora gecmeden once bekle
  }

  Serial.println(F("\n=== Tur tamamlandi, bastan ===\n"));
  delay(2000);
}
