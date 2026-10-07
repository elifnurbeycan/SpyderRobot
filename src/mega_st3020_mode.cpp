/*
 * mega_st3020_mode.cpp — ST3020 operating-mode okuma/yazma (tek seferlik onarim)
 *
 * Sorun: Motor 14 sabit pozisyon komutu (enc=3812) almasina ragmen durmuyor,
 * ters yonde surekli donup mekanik limite dayaniyor. Belirti = wheel/hiz modu.
 * Diger 5 femur ayni komutla duzgun duruyor => sadece 14'un EEPROM modu bozuk.
 *
 * STS/ST3020 kontrol tablosu:
 *   Adres 33 = Operating Mode  (0=pozisyon, 1=hiz/wheel, 2=PWM, 3=step)
 *   Adres 55 = EEPROM Lock     (0=acik, 1=kilitli) — 33 EEPROM'da, yazmadan once acilmali
 *
 * Kullanim (Serial Monitor @115200):
 *   r  -> hattaki motorlarin mode degerini oku
 *   f  -> ID 14'u pozisyon moduna (0) al  [lock ac -> yaz -> lock kapat]
 *   Once 'r' ile 14'un modunu gor, 1 ise 'f' bas, tekrar 'r' ile 0 oldugunu dogrula.
 */

#include <Arduino.h>

#define SERVO_SERIAL   Serial1
#define BAUD           1000000UL

#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_READ      0x02
#define INST_WRITE     0x03

#define ADDR_MODE      33
#define ADDR_LOCK      55
#define ADDR_TORQUE    40

const uint8_t FEMUR_IDS[6] = {2, 5, 8, 11, 14, 17};

void sendPacket(uint8_t id, uint8_t inst, uint8_t *p, uint8_t pLen) {
  uint8_t len = pLen + 2, cs = id + len + inst;
  SERVO_SERIAL.write(HEADER1); SERVO_SERIAL.write(HEADER2);
  SERVO_SERIAL.write(id); SERVO_SERIAL.write(len); SERVO_SERIAL.write(inst);
  for (uint8_t i = 0; i < pLen; i++) { SERVO_SERIAL.write(p[i]); cs += p[i]; }
  SERVO_SERIAL.write((uint8_t)(~cs));
  SERVO_SERIAL.flush();
}

// Tek byte register oku. Ham cerceveyi de basar (ofset dogrulama icin).
int readByte(uint8_t id, uint8_t addr) {
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
  uint8_t p[2] = { addr, 1 };
  sendPacket(id, INST_READ, p, 2);

  uint8_t buf[24], idx = 0;
  unsigned long t = millis();
  while (idx < 24 && millis() - t < 50)
    if (SERVO_SERIAL.available()) buf[idx++] = SERVO_SERIAL.read();
  if (idx < 1) return -1;

  // Ham dump
  Serial.print(F("    ham["));
  Serial.print(idx); Serial.print(F("]:"));
  for (uint8_t i = 0; i < idx; i++) { Serial.print(' '); Serial.print(buf[i]); }
  Serial.println();

  // TX echo'yu atla: gonderdigimiz paket = FF FF id 04 02 addr 01 cs = 8 byte.
  // Yanit ondan sonra: FF FF id len err DATA cs. DATA = echo(8) + 5 = index 13.
  if (idx < 14) return -1;
  return buf[13];
}

void writeByte(uint8_t id, uint8_t addr, uint8_t val) {
  uint8_t p[2] = { addr, val };
  sendPacket(id, INST_WRITE, p, 2);
  delay(50);
}

void okuHepsi() {
  Serial.println(F("--- Operating Mode (adres 33) ---"));
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t id = FEMUR_IDS[i];
    int m = readByte(id, ADDR_MODE);
    Serial.print(F("  ID ")); Serial.print(id); Serial.print(F(" -> mode = "));
    if (m < 0) Serial.println(F("(cevap yok)"));
    else {
      Serial.print(m);
      Serial.print(F("  ("));
      Serial.print(m == 0 ? F("POZISYON [dogru]") :
                   m == 1 ? F("HIZ/WHEEL [YANLIS!]") :
                   m == 2 ? F("PWM") : F("?"));
      Serial.println(F(")"));
    }
  }
}

void poziyonModunaAl(uint8_t id) {
  Serial.print(F("\n>>> ID ")); Serial.print(id);
  Serial.println(F(" pozisyon moduna aliniyor..."));
  Serial.println(F("[0] Torque OFF (motor dursun)"));
  writeByte(id, ADDR_TORQUE, 0);
  delay(100);
  Serial.println(F("[1] EEPROM kilidi aciliyor (lock=0)"));
  writeByte(id, ADDR_LOCK, 0);
  Serial.println(F("[2] mode = 0 yaziliyor"));
  writeByte(id, ADDR_MODE, 0);
  delay(200);
  Serial.println(F("[3] EEPROM kilidi kapatiliyor (lock=1)"));
  writeByte(id, ADDR_LOCK, 1);
  delay(100);
  int m = readByte(id, ADDR_MODE);
  Serial.print(F("[4] Dogrulama: mode = ")); Serial.println(m);
  Serial.println(m == 0 ? F("    BASARILI. Motorun gucunu kapat/ac, sonra test et.")
                        : F("    HALA 0 DEGIL - kilit/adres kontrol et."));
}

#define ADDR_GOAL      42
#define ADDR_PRESENT   56

// 2 byte present position oku (buf ofseti readByte ile ayni mantik, +1 data byte)
int readWord(uint8_t id, uint8_t addr) {
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
  uint8_t p[2] = { addr, 2 };
  sendPacket(id, INST_READ, p, 2);
  uint8_t buf[24], idx = 0;
  unsigned long t = millis();
  while (idx < 24 && millis() - t < 50)
    if (SERVO_SERIAL.available()) buf[idx++] = SERVO_SERIAL.read();
  if (idx < 15) return -1;
  return buf[13] | (buf[14] << 8);   // echo(8)+FF FF id len err + L(13) H(14)
}

void stGoal(uint8_t id, uint16_t pos) {
  uint8_t p[7] = { ADDR_GOAL, (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
                   0, 0, 200, 0 };
  sendPacket(id, INST_WRITE, p, 7);
}

// 14'e hedef ver, present pos'u izle: sabit mi kaliyor kayiyor mu?
void izle(uint8_t id, uint16_t hedef) {
  Serial.print(F("\n>>> ID ")); Serial.print(id);
  Serial.print(F(" -> hedef ")); Serial.println(hedef);
  // once err temizlemek icin torque kapat/ac
  uint8_t off[2] = { ADDR_TORQUE, 0 }; sendPacket(id, INST_WRITE, off, 2); delay(100);
  uint8_t on[2]  = { ADDR_TORQUE, 1 }; sendPacket(id, INST_WRITE, on, 2);  delay(100);
  stGoal(id, hedef);
  for (uint8_t i = 0; i < 12; i++) {
    delay(400);
    int pos = readWord(id, ADDR_PRESENT);
    Serial.print(F("  t=")); Serial.print(i);
    Serial.print(F("  present=")); Serial.println(pos);
  }
}

void setup() {
  Serial.begin(115200);
  SERVO_SERIAL.begin(BAUD);
  delay(500);
  Serial.println(F("ST3020 mode araci hazir."));
  Serial.println(F("  r=mode oku  f=14 pozisyon-mod  t=14 hedef3812 izle"));
  okuHepsi();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'g' || c == 'G') {
    // g <hedef> — ID 14'e istenen enc hedefini ver ve present izle
    long hedef = Serial.parseInt();
    if (hedef < 0) hedef = 0; if (hedef > 4095) hedef = 4095;
    izle(14, (uint16_t)hedef);
    return;
  }
  if (c == 'j' || c == 'J') {
    // j <hedef> — ID 14'e hedef ver, torku ac, TEK present dondur (hizli jog)
    long hedef = Serial.parseInt();
    if (hedef < 0) hedef = 0; if (hedef > 4095) hedef = 4095;
    uint8_t on[2] = { ADDR_TORQUE, 1 }; sendPacket(14, INST_WRITE, on, 2);
    stGoal(14, (uint16_t)hedef);
    delay(350);
    int pos = readWord(14, ADDR_PRESENT);
    Serial.print(F("hedef=")); Serial.print(hedef);
    Serial.print(F(" present=")); Serial.println(pos);
    return;
  }
  if (c == 'd' || c == 'D') {
    // Torku KAPAT — 14 serbest kalir, elle cevirilebilir
    uint8_t off[2] = { ADDR_TORQUE, 0 };
    sendPacket(14, INST_WRITE, off, 2);
    Serial.println(F("torque OFF"));
    return;
  }
  if (c == 'k' || c == 'K') {
    // 14'un present pozisyonunu TEK oku ve dondur (canli izleme icin)
    Serial.println(readWord(14, ADDR_PRESENT));
    return;
  }
  if (c == 'r' || c == 'R') okuHepsi();
  else if (c == 'f' || c == 'F') poziyonModunaAl(14);
  else if (c == 't' || c == 'T') izle(14, 3812);
  else if (c == 'y' || c == 'Y') izle(17, 2994);
  else if (c == 'e' || c == 'E') {
    // Torku kapat, present pos'u 8 sn izle. Sen motoru ELLE cevir.
    // Enkoder saglamsa deger degisir; oluyse sabit/ziplar.
    uint8_t off[2] = { ADDR_TORQUE, 0 };
    sendPacket(14, INST_WRITE, off, 2); delay(100);
    Serial.println(F(">>> ID 14 torque OFF. Motoru ELLE cevir:"));
    for (uint8_t i = 0; i < 20; i++) {
      delay(400);
      Serial.print(F("  present=")); Serial.println(readWord(14, ADDR_PRESENT));
    }
  }
  else if (c == 'l' || c == 'L') {
    // Angle limit (min 9-10, max 11-12) oku: tum femurlari karsilastir
    for (uint8_t i = 0; i < 6; i++) {
      uint8_t id = FEMUR_IDS[i];
      int mn = readWord(id, 9);
      int mx = readWord(id, 11);
      Serial.print(F("ID ")); Serial.print(id);
      Serial.print(F("  min_angle=")); Serial.print(mn);
      Serial.print(F("  max_angle=")); Serial.println(mx);
    }
  }
}
