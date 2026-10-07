/*
 * 4 adet AX-12A hareket testi - Arduino Mega 2560
 *
 * ON KOSUL: Motorlarin ID'leri 1,2,3,4 olarak ayarlanmis olmali.
 *   Ayarlamak icin once mega_ax12_setid ortamini kullan (motorlari TEK TEK bagla).
 *
 * BAGLANTI - Zincirleme (daisy-chain):
 *   AX-12A'nin uzerinde iki konnektor var, ikisi de ayni hatta bagli.
 *   Birinden gir, digerinden cikip sonraki motora git.
 *
 *   Mega TX1 (18) --[1k]--+
 *                         +--- [M1] --- [M2] --- [M3] --- [M4]
 *   Mega RX1 (19) --------+
 *
 *   Guc (+) 11V ve GND tum motorlara ortak gider (zincir uzerinden tasinir).
 *   Mega GND mutlaka guc kaynagi (-) ile ortak olmali.
 *
 * !!! GUC UYARISI !!!
 * 4 AX-12A hareket halinde 4-6A cekebilir. Kalkis aninda daha fazla.
 * Guc kaynagin bunu karsilamiyorsa gerilim duser, motorlar reset atar
 * ya da rastgele davranir. En az 5A onerilir.
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL

#define MOTOR_SAYISI 4
const uint8_t MOTOR_IDS[MOTOR_SAYISI] = { 1, 2, 3, 4 };

// Dynamixel 1.0 protokol sabitleri
#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03

// AX-12A kontrol tablosu adresleri
#define ADDR_TORQUE_ENABLE   24
#define ADDR_GOAL_POSITION   30
#define ADDR_MOVING_SPEED    32

// AX-12A pozisyon limitleri: 0..1023 => 0..300 derece
#define AX12_POS_MAX     1023
#define AX12_POS_CENTER  512

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

void torqueEnable(uint8_t id, bool on)
{
  uint8_t params[2] = { ADDR_TORQUE_ENABLE, (uint8_t)(on ? 1 : 0) };
  sendPacket(id, INST_WRITE, params, 2);
}

void setSpeed(uint8_t id, uint16_t speed)
{
  uint8_t params[3];
  params[0] = ADDR_MOVING_SPEED;
  params[1] = speed & 0xFF;
  params[2] = (speed >> 8) & 0xFF;
  sendPacket(id, INST_WRITE, params, 3);
}

void setPosition(uint8_t id, uint16_t position)
{
  if (position > AX12_POS_MAX) position = AX12_POS_MAX;   // guvenlik siniri

  uint8_t params[3];
  params[0] = ADDR_GOAL_POSITION;
  params[1] = position & 0xFF;
  params[2] = (position >> 8) & 0xFF;
  sendPacket(id, INST_WRITE, params, 3);
}

// Hattaki tum motorlari bul, beklenenlerle karsilastir
void scanAndVerify()
{
  Serial.println(F("--- ID taramasi ---"));

  bool bulundu[MOTOR_SAYISI] = { false, false, false, false };
  uint8_t toplam = 0;

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
      Serial.print(F("  Bulundu -> ID: "));
      Serial.println(id);
      toplam++;

      for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
        if (MOTOR_IDS[i] == id) bulundu[i] = true;
      }
    }
  }

  Serial.print(F("Toplam bulunan: "));
  Serial.println(toplam);

  // Eksik olanlari bildir
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    if (!bulundu[i]) {
      Serial.print(F("  EKSIK -> beklenen ID "));
      Serial.print(MOTOR_IDS[i]);
      Serial.println(F(" cevap vermedi"));
    }
  }

  if (toplam == 0) {
    Serial.println(F("  Kontrol et: GND ortak mi? 11V var mi? DATA hatti dogru mu?"));
  }
  Serial.println(F("--- Tarama bitti ---\n"));
}

void setup()
{
  Serial.begin(115200);
  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);

  Serial.println(F("\n4x AX-12A test basliyor"));

  scanAndVerify();

  // Tum motorlarin torkunu ac ve hizini ayarla
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    torqueEnable(MOTOR_IDS[i], true);
    delay(20);
    setSpeed(MOTOR_IDS[i], 150);          // Yavas, gozle takip edilebilsin
    delay(20);
  }

  Serial.println(F("Tork acildi. Hareket dongusu basliyor.\n"));
}

void loop()
{
  // --- Adim 1: hepsi ortaya ---
  Serial.println(F("-> Hepsi 512 (orta)"));
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    setPosition(MOTOR_IDS[i], AX12_POS_CENTER);
  }
  delay(2000);

  // --- Adim 2: sirayla dalga hareketi ---
  Serial.println(F("-> Sirayla 712 (dalga)"));
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    Serial.print(F("   ID "));
    Serial.println(MOTOR_IDS[i]);
    setPosition(MOTOR_IDS[i], 712);
    delay(400);                           // Sirayla hareket etsinler, ayirt edilsin
  }
  delay(1500);

  // --- Adim 3: hepsi birden geri ---
  Serial.println(F("-> Hepsi birden 312"));
  for (uint8_t i = 0; i < MOTOR_SAYISI; i++) {
    setPosition(MOTOR_IDS[i], 312);
  }
  delay(2000);
}
