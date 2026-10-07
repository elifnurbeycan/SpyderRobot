/*
 * mega_id_tani.cpp - ID cakisma / eslesme tanisi (MOTORLARI OYNATMAZ)
 *
 * AMAC:
 *   "Dagilmis uc bacak kivrildi" sorununun ID cakismasindan mi yoksa
 *   sadece tork-sicramasindan mi geldigini ayirmak.
 *
 *   Bu sketch HICBIR pozisyon komutu gondermez, sadece PING atar.
 *   Motorlar KIPIRDAMAZ - guvenli.
 *
 * NE YAPAR:
 *   Her hat icin ID 1-18'i tek tek pingler, gelen yanit byte sayisini yazar.
 *   - Normal yanit: echo(6 byte) + status(6 byte) = ~12 byte
 *   - CAKISMA: ayni ID'de iki motor varsa yanitlar cakisir, byte sayisi
 *     bozuk/fazla cikar ya da hic gelmez (bus collision).
 *   - Beklenen ID'ler motor_ids.h'a gore isaretlenir.
 *
 * BAGLANTI:
 *   ST3020 (femur) : Serial1 (TX1=18, RX1=19)
 *   AX-12A (coxa/tibia) : Serial2 (TX2=16, RX2=17)
 *
 * KOMUT: 's' + Enter = tekrar tara
 */

#include <Arduino.h>
#include "motor_ids.h"

#define HEADER1   0xFF
#define HEADER2   0xFF
#define INST_PING 0x01

// Bir ID beklenen listede mi? (etiketiyle birlikte dondurur)
const char* beklenenEtiket(uint8_t id)
{
  for (uint8_t i = 0; i < BACAK_SAYISI; i++) {
    if (FEMUR_IDS[i] == id) return "FEMUR (ST3020)";
    if (COXA_IDS[i]  == id) return "COXA  (AX-12A)";
    if (TIBIA_IDS[i] == id) return "TIBIA (AX-12A)";
  }
  return NULL;
}

// PING at, gelen toplam byte sayisini dondur (echo dahil)
uint8_t pingByteSay(HardwareSerial &ser, uint8_t id)
{
  while (ser.available()) ser.read();

  uint8_t checksum = (uint8_t)(~(id + 0x02 + INST_PING));
  ser.write(HEADER1); ser.write(HEADER2);
  ser.write(id);      ser.write((uint8_t)0x02);
  ser.write(INST_PING); ser.write(checksum);
  ser.flush();

  unsigned long t = millis();
  uint8_t n = 0;
  while (millis() - t < 20) {
    if (ser.available()) { ser.read(); n++; }
  }
  return n;
}

void tara(HardwareSerial &ser, const char *hatAdi)
{
  Serial.print(F("\n--- HAT: ")); Serial.print(hatAdi);
  Serial.println(F(" (ID 1-18) ---"));
  Serial.println(F("ID  | byte | durum"));

  for (uint8_t id = 1; id <= 18; id++) {
    uint8_t n = pingByteSay(ser, id);
    const char *etiket = beklenenEtiket(id);

    Serial.print(F("  "));
    if (id < 10) Serial.print(' ');
    Serial.print(id);   Serial.print(F("  |  "));
    if (n < 10) Serial.print(' ');
    Serial.print(n);    Serial.print(F("  | "));

    if (n <= 6) {
      Serial.print(F("cevap YOK"));
    } else if (n >= 10 && n <= 14) {
      Serial.print(F("OK"));
    } else {
      Serial.print(F("!! ANORMAL (cakisma?)"));
    }

    if (etiket) { Serial.print(F("  <- beklenen: ")); Serial.print(etiket); }
    else if (n > 6) { Serial.print(F("  <- BEKLENMEYEN ID cevap verdi!")); }

    Serial.println();
  }
}

void tumTara()
{
  Serial.println(F("\n======== ID TANI (motorlar oynamaz) ========"));
  tara(FEMUR_SERIAL, "Serial1 / ST3020 (femur bekleniyor)");
  tara(AX_SERIAL,    "Serial2 / AX-12A (coxa+tibia bekleniyor)");
  Serial.println(F("\nBeklenen: FEMUR ID'leri Serial1'de, COXA+TIBIA Serial2'de OK."));
  Serial.println(F("ANORMAL byte = ayni ID'de birden fazla motor olabilir."));
  Serial.println(F("\n's' ile tekrar tara."));
}

void setup()
{
  Serial.begin(115200);
  FEMUR_SERIAL.begin(FEMUR_BAUD);
  AX_SERIAL.begin(AX_BAUD);
  delay(500);
  tumTara();
}

void loop()
{
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\r' || c == '\n') return;
  if (c == 's' || c == 'S') tumTara();
}
