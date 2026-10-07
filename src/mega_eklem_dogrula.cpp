/*
 * mega_eklem_dogrula.cpp - Eklem ID eslemesi gozle dogrulama - Arduino Mega 2560
 *
 * AMAC:
 *   motor_ids.h icindeki coxa/femur/tibia eslemesi OLCULMUS DEGIL, elle
 *   girilmis bir bilgi. Bu sketch her eklemi TEK TEK oynatir, sen ekrana
 *   bakip dogru eklemin kipirdadigini teyit edersin.
 *
 *   Ozellikle onemli: bacak 1-3 ile bacak 4-6 arasinda eklem sirasi TERS.
 *   (bacak 1: tibia=1, coxa=3   |   bacak 4: coxa=10, tibia=12)
 *   Hata olacaksa buyuk ihtimalle oradadir.
 *
 * IKI PROTOKOL:
 *   FEMUR -> ST3020 (Feetech STS) : Serial1, pozisyon 0..4095, merkez 2048
 *   COXA/TIBIA -> AX-12A (Dynamixel) : Serial2, pozisyon 0..1023, merkez 512
 *   Paket formatlari benzer ama adresler ve pozisyon araliklari FARKLI.
 *
 * BAGLANTI:
 *   Mega TX1 (18) --[2.2k]--+--- ST3020 zinciri (femurlar)
 *   Mega RX1 (19) ----------+
 *   Mega TX2 (16) --[1k]----+--- AX-12A zinciri (coxa + tibia)
 *   Mega RX2 (17) ----------+
 *   Mega GND ---- guc kaynagi (-) ile ORTAK olmali
 *
 * !!! GERILIM !!! ST3020 6-14V ister, 5V calismaz. Besleme 11-12V olmali.
 *
 * !!! GUVENLIK !!!
 *   Bu sketch ayni anda TEK motor oynatir - akim dusuk kalir.
 *   Hareket acisi kucuk (COXA/TIBIA 15, FEMUR 10 derece) - montajli
 *   sistemde carpma riskini azaltir. Robot ASKIDA/YAN YATIRILMIS olsun,
 *   ayakta durursa kendi agirligiyla devrilir.
 *
 * KOMUTLAR (Enter ile gonder):
 *   1-6  - sadece o bacagi test et
 *   a    - tum bacaklari sirayla test et
 *   t    - tum torklari kapat (motorlar serbest kalir)
 */

#include <Arduino.h>
#include "motor_ids.h"

// --- Protokol sabitleri (her iki tip icin ortak) ---
#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_WRITE     0x03

// --- ST3020 (femur) kontrol tablosu ---
#define STS_ADDR_TORQUE     40
#define STS_ADDR_GOAL_POS   42
#define STS_POS_MAX         4095
#define STS_POS_CENTER      2048
#define STS_ADIM_PER_DERECE 11.378f     // 4096 adim / 360 derece
#define STS_HIZ             500

// --- AX-12A (coxa, tibia) kontrol tablosu ---
#define AX_ADDR_TORQUE      24
#define AX_ADDR_GOAL_POS    30
#define AX_ADDR_SPEED       32
#define AX_POS_MAX          1023
#define AX_POS_CENTER       512
#define AX_ADIM_PER_DERECE  3.41f       // 1024 adim / 300 derece
#define AX_HIZ              150

// --- Test hareketi ---
#define FEMUR_ACI     10                // derece, femur daha az oynasin
#define AX_ACI        15                // derece, coxa/tibia
#define BEKLE_MS      700               // her hareket arasi gozle takip suresi

// Eklem tipleri
enum EklemTipi { EKLEM_COXA, EKLEM_FEMUR, EKLEM_TIBIA };

void sendPacket(HardwareSerial &ser, uint8_t id, uint8_t instruction,
                uint8_t *params, uint8_t paramLen)
{
  uint8_t length = paramLen + 2;
  uint8_t checksum = id + length + instruction;

  ser.write(HEADER1);
  ser.write(HEADER2);
  ser.write(id);
  ser.write(length);
  ser.write(instruction);

  for (uint8_t i = 0; i < paramLen; i++) {
    ser.write(params[i]);
    checksum += params[i];
  }

  ser.write((uint8_t)(~checksum));
  ser.flush();
}

// --- ST3020 (femur) ---
void stsTorque(uint8_t id, bool on)
{
  uint8_t params[2] = { STS_ADDR_TORQUE, (uint8_t)(on ? 1 : 0) };
  sendPacket(FEMUR_SERIAL, id, INST_WRITE, params, 2);
}

void stsSetAci(uint8_t id, float derece)
{
  long hedef = (long)STS_POS_CENTER + (long)(derece * STS_ADIM_PER_DERECE);
  if (hedef < 0) hedef = 0;                          // guvenlik siniri
  if (hedef > STS_POS_MAX) hedef = STS_POS_MAX;

  uint8_t params[7];
  params[0] = STS_ADDR_GOAL_POS;
  params[1] = (uint16_t)hedef & 0xFF;
  params[2] = ((uint16_t)hedef >> 8) & 0xFF;
  params[3] = 0;                                     // adres 44-45: time
  params[4] = 0;
  params[5] = STS_HIZ & 0xFF;
  params[6] = (STS_HIZ >> 8) & 0xFF;
  sendPacket(FEMUR_SERIAL, id, INST_WRITE, params, 7);
}

// --- AX-12A (coxa, tibia) ---
void axTorque(uint8_t id, bool on)
{
  uint8_t params[2] = { AX_ADDR_TORQUE, (uint8_t)(on ? 1 : 0) };
  sendPacket(AX_SERIAL, id, INST_WRITE, params, 2);
}

void axSetSpeed(uint8_t id, uint16_t speed)
{
  uint8_t params[3];
  params[0] = AX_ADDR_SPEED;
  params[1] = speed & 0xFF;
  params[2] = (speed >> 8) & 0xFF;
  sendPacket(AX_SERIAL, id, INST_WRITE, params, 3);
}

void axSetAci(uint8_t id, float derece)
{
  long hedef = (long)AX_POS_CENTER + (long)(derece * AX_ADIM_PER_DERECE);
  if (hedef < 0) hedef = 0;                          // guvenlik siniri
  if (hedef > AX_POS_MAX) hedef = AX_POS_MAX;

  uint8_t params[3];
  params[0] = AX_ADDR_GOAL_POS;
  params[1] = (uint16_t)hedef & 0xFF;
  params[2] = ((uint16_t)hedef >> 8) & 0xFF;
  sendPacket(AX_SERIAL, id, INST_WRITE, params, 3);
}

// Tek eklemi merkez -> +aci -> merkez -> -aci -> merkez seklinde oynat
void eklemOynat(uint8_t bacak, EklemTipi tip)
{
  uint8_t  id;
  float    aci;
  const char *ad;
  bool     femurMu = (tip == EKLEM_FEMUR);

  switch (tip) {
    case EKLEM_COXA:  id = COXA_IDS[bacak];  ad = "COXA ";  aci = AX_ACI;    break;
    case EKLEM_FEMUR: id = FEMUR_IDS[bacak]; ad = "FEMUR";  aci = FEMUR_ACI; break;
    default:          id = TIBIA_IDS[bacak]; ad = "TIBIA";  aci = AX_ACI;    break;
  }

  Serial.print(F("  Bacak "));   Serial.print(bacak + 1);
  Serial.print(F(" | "));        Serial.print(ad);
  Serial.print(F(" | ID "));     Serial.print(id);
  Serial.print(F(" | "));        Serial.print(femurMu ? F("ST3020") : F("AX-12A"));
  Serial.println(F("  <- bu eklem oynamali"));

  // merkez -> +aci -> merkez -> -aci -> merkez
  const float adimlar[5] = { 0, +aci, 0, -aci, 0 };
  for (uint8_t i = 0; i < 5; i++) {
    if (femurMu) stsSetAci(id, adimlar[i]);
    else         axSetAci(id, adimlar[i]);
    delay(BEKLE_MS);
  }
}

void bacakTest(uint8_t bacak)
{
  Serial.print(F("\n=== BACAK "));
  Serial.print(bacak + 1);
  Serial.println(F(" ==="));

  eklemOynat(bacak, EKLEM_COXA);
  eklemOynat(bacak, EKLEM_FEMUR);
  eklemOynat(bacak, EKLEM_TIBIA);
}

void tumTorkAc()
{
  for (uint8_t i = 0; i < BACAK_SAYISI; i++) {
    stsTorque(FEMUR_IDS[i], true);  delay(20);
    axTorque(COXA_IDS[i], true);    delay(20);
    axTorque(TIBIA_IDS[i], true);   delay(20);
    axSetSpeed(COXA_IDS[i], AX_HIZ);  delay(20);
    axSetSpeed(TIBIA_IDS[i], AX_HIZ); delay(20);
  }
}

void tumTorkKapat()
{
  for (uint8_t i = 0; i < BACAK_SAYISI; i++) {
    stsTorque(FEMUR_IDS[i], false); delay(20);
    axTorque(COXA_IDS[i], false);   delay(20);
    axTorque(TIBIA_IDS[i], false);  delay(20);
  }
  Serial.println(F("\nTum torklar KAPATILDI - motorlar serbest."));
}

void setup()
{
  Serial.begin(115200);
  FEMUR_SERIAL.begin(FEMUR_BAUD);
  AX_SERIAL.begin(AX_BAUD);
  delay(500);

  Serial.println(F("\n==== EKLEM ESLEME DOGRULAMA ===="));
  Serial.println(F("FEMUR      : ST3020 @ Serial1"));
  Serial.println(F("COXA/TIBIA : AX-12A @ Serial2"));
  Serial.println(F("\n!! Robot askida/yan yatirilmis olmali !!"));
  Serial.println(F("\nHer eklem tek tek oynar. Ekrandaki yazi ile"));
  Serial.println(F("gercekte kipirdayan eklemi karsilastir.\n"));
  Serial.println(F("Komutlar: 1-6 = tek bacak | a = hepsi | t = tork kapat"));

  tumTorkAc();
  Serial.println(F("\nTork acildi. Komut bekleniyor..."));
}

void loop()
{
  if (!Serial.available()) return;

  char c = Serial.read();
  if (c == '\r' || c == '\n') return;        // satir sonlarini yoksay

  if (c >= '1' && c <= '6') {
    bacakTest(c - '1');
    Serial.println(F("\nBitti. Komut bekleniyor..."));
  }
  else if (c == 'a' || c == 'A') {
    for (uint8_t i = 0; i < BACAK_SAYISI; i++) bacakTest(i);
    Serial.println(F("\nTum bacaklar bitti. Komut bekleniyor..."));
  }
  else if (c == 't' || c == 'T') {
    tumTorkKapat();
  }
}
