/*
 * ST3020 akim testi - 3 motor ES ZAMANLI hareket - Arduino Mega 2560
 *
 * AMAC: Hareket halinde gerilim cokmesi var mi olcmek.
 * Motorlar SIRAYLA degil, AYNI ANDA hareket eder - en kotu durum akimi.
 *
 * Baslangicta 3 saniye bekler (multimetreyi hazirla), sonra 10 saniye
 * boyunca surekli +10/-10 derece salinim yapar, sonra durur.
 *
 * ID'ler: 2, 11, 17
 *
 * BAGLANTI:
 *   Mega TX1 (pin 18) --[2.2k]--+--- ST3020 DATA zinciri
 *   Mega RX1 (pin 19) ----------+
 *   Mega GND ------------------------ ST3020 GND ---- Guc (-)
 *   Guc (+) 11V --------------------- ST3020 VCC
 *
 * OLCUM: Hareket sirasinda dusurucu cikisinda voltaji izle.
 *   11V'ta kaliyorsa -> guc yeterli, sorun sinyal butunlugunde
 *   8-9V'a dusuyorsa  -> guc darbogazi
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL

#define MOTOR_SAYISI 3
const uint8_t MOTOR_IDS[MOTOR_SAYISI] = { 2, 11, 17 };

#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03

#define ADDR_TORQUE_ENABLE   40
#define ADDR_GOAL_POSITION   42

#define STS_POS_MAX      4095
#define STS_POS_CENTER   2048
#define ADIM_PER_DERECE  11.378f

#define TEST_ACI_DERECE  10
#define TEST_HIZ         800        // Orta hiz - akim ceksin ama sert olmasin

#define HAZIRLIK_SN      3          // Multimetreyi hazirlama suresi
#define TEST_SURE_SN     10         // Salinim suresi

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
    if (SERVO_SERIAL.available()) { SERVO_SERIAL.read(); bytesIn++; }
  }
  return (bytesIn > 6);       // 6 byte = kendi echo'muz
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
  params[3] = 0;
  params[4] = 0;
  params[5] = speed & 0xFF;
  params[6] = (speed >> 8) & 0xFF;
  sendPacket(id, INST_WRITE, params, 7);
}

void setAci(uint8_t id, float derece, uint16_t speed)
{
  long hedef = (long)STS_POS_CENTER + (long)(derece * ADIM_PER_DERECE);
  if (hedef < 0) hedef = 0;                       // guvenlik siniri
  if (hedef > STS_POS_MAX) hedef = STS_POS_MAX;
  setPosition(id, (uint16_t)hedef, speed);
}

// Tum motorlari AYNI ANDA hedefe gonder - es zamanli akim cekisi
void hepsiniAyniAndaGonder(float derece)
{
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    setAci(MOTOR_IDS[i], derece, TEST_HIZ);
  }
}

void setup()
{
  Serial.begin(115200);
  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);

  Serial.println(F("\n=== ST3020 AKIM TESTI (3 motor es zamanli) ==="));

  // --- Tarama ---
  Serial.println(F("\nTarama:"));
  uint8_t bulunan = 0;
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    if (pingServo(MOTOR_IDS[i])) {
      Serial.print(F("  OK    -> ID "));
      Serial.println(MOTOR_IDS[i]);
      bulunan++;
    } else {
      Serial.print(F("  EKSIK -> ID "));
      Serial.println(MOTOR_IDS[i]);
    }
  }

  if (bulunan == 0) {
    Serial.println(F("\nHicbir motor yok. Test yapilamaz."));
    return;
  }

  // --- Tork ac, merkeze al ---
  Serial.println(F("\nTork aciliyor, merkeze aliniyor..."));
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    torqueEnable(MOTOR_IDS[i], true);
    delay(20);
  }
  hepsiniAyniAndaGonder(0);
  delay(1500);

  // --- Hazirlik geri sayimi ---
  Serial.print(F("\nMultimetreyi hazirla. Test "));
  Serial.print(HAZIRLIK_SN);
  Serial.println(F(" saniye sonra basliyor:"));
  for (uint8_t sn = HAZIRLIK_SN; sn > 0; sn--) {
    Serial.print(F("  "));
    Serial.println(sn);
    delay(1000);
  }

  // --- 10 saniye salinim ---
  Serial.println(F("\n>>> HAREKET BASLADI - VOLTAJI IZLE <<<\n"));

  unsigned long testBas = millis();
  bool yukari = true;

  while (millis() - testBas < (unsigned long)TEST_SURE_SN * 1000UL) {
    if (yukari) {
      Serial.println(F("  +10 derece"));
      hepsiniAyniAndaGonder(+TEST_ACI_DERECE);
    } else {
      Serial.println(F("  -10 derece"));
      hepsiniAyniAndaGonder(-TEST_ACI_DERECE);
    }
    yukari = !yukari;
    delay(700);                     // Salinim periyodu
  }

  // --- Bitir: merkeze al, torku birak ---
  Serial.println(F("\n>>> TEST BITTI <<<"));
  hepsiniAyniAndaGonder(0);
  delay(1000);

  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    torqueEnable(MOTOR_IDS[i], false);   // Torku birak, isinmasin
    delay(20);
  }

  Serial.println(F("Tork birakildi. Olctugun voltaji soyle."));
}

void loop()
{
  // Tek seferlik test, loop bos
}
