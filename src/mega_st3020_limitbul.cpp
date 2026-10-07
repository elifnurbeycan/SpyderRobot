/*
 * ST3020 Mekanik Limit Bulucu - Arduino Mega 2560
 *
 * Her motoru sirayla her iki yone yavascac surer.
 * Pozisyon donunca (motor sikisinc) durur ve o anki deger limit olarak kaydedilir.
 *
 * FEMUR MOTORLARI: ID 2, 5, 8, 11, 14, 17
 *
 * KOMUTLAR (seri monitor):
 *   t   - tum motorlari otomatik tara
 *   <sayi>  - sadece o ID'li motoru tara
 *   s   - tara (ping) - hangi motorlar var
 *
 * GUVENLIK:
 *   - Motor pozisyon degismezse (STALL_THRESHOLD adim icinde) durur
 *   - Her hareket kucuk adimlarla (ADIM_BUYUKLUGU) yapilir
 *   - Maksimum sure asiminda (TIMEOUT_MS) zorla durur
 *
 * BAGLANTI:
 *   Mega TX1 (pin 18) --[1k]--+--- ST3020 DATA
 *   Mega RX1 (pin 19) --------+
 */

#include <Arduino.h>

// ---------- Seri port ----------
#define SERVO_SERIAL  Serial1
#define SERVO_BAUD    1000000UL

// ---------- Protokol ----------
#define HEADER1           0xFF
#define HEADER2           0xFF
#define INST_PING         0x01
#define INST_READ         0x02
#define INST_WRITE        0x03

// ---------- Kontrol tablosu adresleri ----------
#define ADDR_TORQUE_ENABLE   40
#define ADDR_GOAL_POSITION   42   // 2 byte, little-endian
#define ADDR_PRESENT_POS     56   // 2 byte, little-endian
#define ADDR_PRESENT_LOAD    60   // 2 byte

// ---------- Sabitler ----------
#define STS_POS_MAX      4095
#define STS_POS_CENTER   2048

// Tarama parametreleri
#define ADIM_BUYUKLUGU   15      // Her hamlede kac adim (kucuk = guvenli ama yavas)
#define ADIM_GECIKMESI   60      // Her adim arasi bekleme (ms)
#define STALL_THRESHOLD  8       // Bu kadar ardisik pozisyon degismezse "sikismis" say
#define TIMEOUT_MS       8000UL  // Tek yon icin maksimum sure (ms)
#define MERKEZ_BEKLEME   1500    // Merkeze donunce bekleme (ms)

// ---------- Femur motor ID'leri ----------
const uint8_t FEMUR_IDS[]  = {2, 5, 8, 11, 14, 17};
const uint8_t FEMUR_SAYI   = sizeof(FEMUR_IDS);

// Bulunan limitler
int16_t limitMin[6];
int16_t limitMax[6];

// ---------- Paket gonderme ----------
void sendPacket(uint8_t id, uint8_t inst, uint8_t *params, uint8_t pLen)
{
  uint8_t length   = pLen + 2;
  uint8_t checksum = id + length + inst;

  SERVO_SERIAL.write(HEADER1);
  SERVO_SERIAL.write(HEADER2);
  SERVO_SERIAL.write(id);
  SERVO_SERIAL.write(length);
  SERVO_SERIAL.write(inst);

  for (uint8_t i = 0; i < pLen; i++) {
    SERVO_SERIAL.write(params[i]);
    checksum += params[i];
  }
  SERVO_SERIAL.write((uint8_t)(~checksum));
  SERVO_SERIAL.flush();
}

// ---------- Ping ----------
bool pingServo(uint8_t id)
{
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();
  sendPacket(id, INST_PING, NULL, 0);

  unsigned long t = millis();
  uint8_t n = 0;
  while (millis() - t < 20) {
    if (SERVO_SERIAL.available()) { SERVO_SERIAL.read(); n++; }
  }
  return (n > 6);
}

// ---------- Pozisyon oku ----------
// Half-duplex: TX gonderidigimizde ayni byte'lar RX'te echo olarak gelir.
// TX paketi 8 byte -> once 8 echo byte'i atla, sonra motorun 8 byte yanitini oku.
//
// TX paketi: FF FF id 04 02 posAddr 02 checksum  (8 byte)
// Motor yaniti: FF FF id 04 err posL posH checksum  (8 byte)
// Toplam beklenen: 16 byte, ilk 8 = echo, son 8 = gercek yanit
int16_t readPosition(uint8_t id)
{
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();

  uint8_t params[2] = { ADDR_PRESENT_POS, 2 };
  sendPacket(id, INST_READ, params, 2);

  // 16 byte oku: [0..7] = echo, [8..15] = motor yaniti
  uint8_t buf[16];
  uint8_t idx = 0;
  unsigned long t = millis();
  while (idx < 16 && millis() - t < 50) {
    if (SERVO_SERIAL.available()) buf[idx++] = SERVO_SERIAL.read();
  }
  if (idx < 16) return -1;  // zaman asimi veya motor cevap vermedi

  // Motor yaniti: buf[8]=FF buf[9]=FF buf[10]=id buf[11]=len buf[12]=err buf[13]=posL buf[14]=posH buf[15]=checksum
  int16_t pos = (int16_t)(buf[13] | ((uint16_t)buf[14] << 8));
  return pos;
}

// ---------- Pozisyon yaz ----------
void setPosition(uint8_t id, uint16_t pos, uint16_t spd)
{
  if (pos > STS_POS_MAX) pos = STS_POS_MAX;
  uint8_t params[7];
  params[0] = ADDR_GOAL_POSITION;
  params[1] = pos & 0xFF;
  params[2] = (pos >> 8) & 0xFF;
  params[3] = 0;
  params[4] = 0;
  params[5] = spd & 0xFF;
  params[6] = (spd >> 8) & 0xFF;
  sendPacket(id, INST_WRITE, params, 7);
}

// ---------- Tork ----------
void torqueEnable(uint8_t id, bool on)
{
  uint8_t p[2] = { ADDR_TORQUE_ENABLE, (uint8_t)(on ? 1 : 0) };
  sendPacket(id, INST_WRITE, p, 2);
}

// ---------- Tek motor limit bul ----------
// yon: +1 = artan pozisyon (CW), -1 = azalan (CCW)
// Donus degeri: bulunan limit pozisyonu, hata durumunda -1
int16_t limitTara(uint8_t id, int8_t yon)
{
  int16_t oncekiPos = readPosition(id);
  if (oncekiPos < 0) {
    Serial.println(F("    HATA: Pozisyon okunamadi!"));
    return -1;
  }

  int16_t hedef = oncekiPos;
  uint8_t durmaySayaci = 0;
  unsigned long baslangic = millis();

  Serial.print(F("    Baslangic pozisyon: "));
  Serial.println(oncekiPos);

  while (millis() - baslangic < TIMEOUT_MS) {
    // Yeni hedef
    hedef += (int16_t)(yon * ADIM_BUYUKLUGU);
    if (hedef < 0)         hedef = 0;
    if (hedef > STS_POS_MAX) hedef = STS_POS_MAX;

    setPosition(id, (uint16_t)hedef, 200);  // dusuk hiz
    delay(ADIM_GECIKMESI);

    int16_t simdikiPos = readPosition(id);
    if (simdikiPos < 0) continue;  // okuma hatasi, devam et

    int16_t fark = abs(simdikiPos - oncekiPos);

    Serial.print(F("      pos="));
    Serial.print(simdikiPos);
    Serial.print(F(" hedef="));
    Serial.print(hedef);
    Serial.print(F(" fark="));
    Serial.println(fark);

    if (fark < 4) {
      // Pozisyon neredeyse degismedi
      durmaySayaci++;
      if (durmaySayaci >= STALL_THRESHOLD) {
        Serial.print(F("    >>> Limit bulundu: "));
        Serial.println(simdikiPos);
        return simdikiPos;
      }
    } else {
      durmaySayaci = 0;
    }

    oncekiPos = simdikiPos;

    // Kenar kontrolu
    if (hedef <= 0 || hedef >= STS_POS_MAX) {
      Serial.println(F("    Pozisyon sinira ulasti (mekanik limit yok?)"));
      return simdikiPos;
    }
  }

  Serial.println(F("    UYARI: Sure asimi! Son pozisyon limit kabul edildi."));
  return readPosition(id);
}

// ---------- Bir motoru tara ----------
void motoruTara(uint8_t id, uint8_t idx)
{
  Serial.println();
  Serial.print(F("========== ID "));
  Serial.print(id);
  Serial.println(F(" taranıyor =========="));

  if (!pingServo(id)) {
    Serial.println(F("  HATA: Motor cevap vermiyor, atlaniyor."));
    limitMin[idx] = -1;
    limitMax[idx] = -1;
    return;
  }

  torqueEnable(id, true);
  delay(100);

  // --- Merkeze getir ---
  Serial.println(F("  [1] Merkeze (2048) getiriliyor..."));
  setPosition(id, STS_POS_CENTER, 300);
  delay(MERKEZ_BEKLEME);

  // --- MIN limit (CCW, azalan yon) ---
  Serial.println(F("  [2] MIN limit araniyor (azalan yon)..."));
  int16_t lMin = limitTara(id, -1);
  limitMin[idx] = lMin;

  // --- Merkeze don ---
  Serial.println(F("  [3] Merkeze donuluyor..."));
  setPosition(id, STS_POS_CENTER, 300);
  delay(MERKEZ_BEKLEME);

  // --- MAX limit (CW, artan yon) ---
  Serial.println(F("  [4] MAX limit araniyor (artan yon)..."));
  int16_t lMax = limitTara(id, +1);
  limitMax[idx] = lMax;

  // --- Merkeze don ---
  Serial.println(F("  [5] Merkeze donuluyor..."));
  setPosition(id, STS_POS_CENTER, 300);
  delay(MERKEZ_BEKLEME);

  torqueEnable(id, false);

  Serial.print(F("  SONUC => MIN: "));
  Serial.print(lMin);
  Serial.print(F("  MAX: "));
  Serial.print(lMax);
  if (lMin >= 0 && lMax >= 0) {
    Serial.print(F("  ARALIK: "));
    Serial.print(lMax - lMin);
    Serial.print(F(" adim (~"));
    Serial.print((float)(lMax - lMin) * 360.0f / 4096.0f, 1);
    Serial.println(F(" derece)"));
  } else {
    Serial.println();
  }
}

// ---------- Tum motorlari tara ----------
void tumunuTara()
{
  Serial.println(F("\n===== TUM FEMUR MOTORLARI TARANACAK ====="));
  Serial.println(F("Motorlar hareket edecek! Dikkatli ol.\n"));
  delay(2000);

  for (uint8_t i = 0; i < FEMUR_SAYI; i++) {
    motoruTara(FEMUR_IDS[i], i);
  }

  // Ozet tablosu
  Serial.println(F("\n========== OZET TABLOSU =========="));
  Serial.println(F("ID  | MIN   | MAX   | ARALIK | DERECE"));
  Serial.println(F("----|-------|-------|--------|-------"));
  for (uint8_t i = 0; i < FEMUR_SAYI; i++) {
    Serial.print(FEMUR_IDS[i]);
    Serial.print(F("   | "));
    if (limitMin[i] < 0) {
      Serial.println(F("HATA  | HATA  | -      | -"));
    } else {
      int16_t aralik = limitMax[i] - limitMin[i];
      float   derece = aralik * 360.0f / 4096.0f;
      Serial.print(limitMin[i]);
      Serial.print(F(" | "));
      Serial.print(limitMax[i]);
      Serial.print(F(" | "));
      Serial.print(aralik);
      Serial.print(F("  | "));
      Serial.println(derece, 1);
    }
  }
  Serial.println(F("===================================\n"));
}

// ---------- Tarama (ping) ----------
void taraPing()
{
  Serial.println(F("\n>>> Tarama"));
  for (uint16_t id = 0; id < 254; id++) {
    if (pingServo((uint8_t)id)) {
      Serial.print(F("  Bulundu -> ID: "));
      Serial.println(id);
    }
  }
  Serial.println();
}

// ---------- Yardim ----------
void yardim()
{
  Serial.println(F("\n=== ST3020 MEKANIK LIMIT BULUCU ==="));
  Serial.println(F("Komutlar:"));
  Serial.println(F("  t         - tum femur motorlarini tara (ID: 2,5,8,11,14,17)"));
  Serial.println(F("  <sayi>    - sadece o ID'li motoru tara"));
  Serial.println(F("  s         - hangi motorlar var (ping tarama)"));
  Serial.println(F("\nUYARI: Motorlar hareket edecek! Bacaklarin acik olduguna emin ol.\n"));
}

// ---------- Setup ----------
void setup()
{
  Serial.begin(115200);
  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);
  yardim();
  Serial.println(F("Komut bekleniyor..."));
}

// ---------- Loop ----------
void loop()
{
  if (!Serial.available()) return;

  String girdi = Serial.readStringUntil('\n');
  girdi.trim();
  if (girdi.length() == 0) return;

  if (girdi == "t" || girdi == "T") {
    tumunuTara();
    return;
  }

  if (girdi == "s" || girdi == "S") {
    taraPing();
    return;
  }

  long deger = girdi.toInt();
  if (deger >= 0 && deger <= 253) {
    // Hangi index?
    uint8_t idx = 0;
    for (uint8_t i = 0; i < FEMUR_SAYI; i++) {
      if (FEMUR_IDS[i] == (uint8_t)deger) { idx = i; break; }
    }
    motoruTara((uint8_t)deger, idx);
  } else {
    Serial.print(F("Gecersiz: "));
    Serial.println(girdi);
    yardim();
  }
}
