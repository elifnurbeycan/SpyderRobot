/*
 * AX-12A interaktif ID atama araci - Arduino Mega 2560
 *
 * BIR KEZ YUKLE, 12 MOTORU SIRAYLA GEC.
 * Kod degistirip yeniden yuklemeye gerek yok - ID'yi seri porttan gonderiyorsun.
 *
 * KULLANIM:
 *   1. Bu kodu bir kez yukle
 *   2. Seri monitoru ac (115200)
 *   3. Hatta SADECE bir motor bagla
 *   4. Monitore atamak istedigin ID'yi yaz + Enter (ornek: 5)
 *   5. Motoru cikar, sirakini bagla, yeni ID yaz
 *   ... 12 motor bitene kadar tekrarla
 *
 * KOMUTLAR:
 *   <sayi>   1-253 arasi: hattaki tek motora bu ID'yi ata
 *   s        tarama yap, hatta kim var goster
 *   t        bulunan tum motorlari kisaca oynat (dogrulama)
 *
 * GUVENLIK:
 *   Hatta birden fazla motor varsa ID YAZILMAZ. Broadcast ile yazildigi
 *   icin hepsi ayni ID'yi alirdi - kod bunu engelliyor.
 *
 * BAGLANTI:
 *   Mega TX1 (pin 18) --[1k]--+--- AX-12A DATA
 *   Mega RX1 (pin 19) --------+
 *   Mega GND ----------------------- AX-12A GND ---- Guc (-)
 *   Guc (+) 11V -------------------- AX-12A VCC
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL

#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03
#define BROADCAST_ID   0xFE     // 254 = hattaki herkes

// AX-12A kontrol tablosu adresleri
#define ADDR_ID              3
#define ADDR_TORQUE_ENABLE   24
#define ADDR_GOAL_POSITION   30
#define ADDR_MOVING_SPEED    32

#define AX12_POS_CENTER  512

// Taramada bulunan ID'ler
uint8_t bulunanIDs[32];
uint8_t bulunanSayi = 0;

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

// Tek bir ID'ye ping at, cevap var mi?
bool pingServo(uint8_t id)
{
  while (SERVO_SERIAL.available()) SERVO_SERIAL.read();   // tamponu temizle

  sendPacket(id, INST_PING, NULL, 0);

  unsigned long start = millis();
  uint8_t bytesIn = 0;
  while (millis() - start < 20) {
    if (SERVO_SERIAL.available()) {
      SERVO_SERIAL.read();
      bytesIn++;
    }
  }

  // Kendi gonderdigimiz 6 byte half-duplex hatta geri okunur.
  // Fazlasi gercek cevap demektir.
  return (bytesIn > 6);
}

// Tum hatti tara, bulunanlari global diziye yaz
uint8_t tara(bool sessiz)
{
  bulunanSayi = 0;

  for (uint16_t id = 0; id < 254; id++) {
    if (pingServo((uint8_t)id)) {
      if (bulunanSayi < 32) {
        bulunanIDs[bulunanSayi] = (uint8_t)id;
      }
      bulunanSayi++;

      if (!sessiz) {
        Serial.print(F("  Bulundu -> ID: "));
        Serial.println(id);
      }
    }
  }

  if (!sessiz && bulunanSayi == 0) {
    Serial.println(F("  (hicbir motor cevap vermedi)"));
  }
  return bulunanSayi;
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
  if (position > 1023) position = 1023;   // guvenlik siniri

  uint8_t params[3];
  params[0] = ADDR_GOAL_POSITION;
  params[1] = position & 0xFF;
  params[2] = (position >> 8) & 0xFF;
  sendPacket(id, INST_WRITE, params, 3);
}

// ---------------------------------------------------------------
// ID atama. Hatta tek motor olmasi sart.
// ---------------------------------------------------------------
void idAta(uint8_t yeniID)
{
  Serial.print(F("\n>>> ID "));
  Serial.print(yeniID);
  Serial.println(F(" atanacak"));

  // --- Once hatta kac motor var? ---
  Serial.println(F("[1] Hat kontrolu..."));
  uint8_t adet = tara(true);

  if (adet == 0) {
    Serial.println(F("    HATA: Hicbir motor yok. ID yazilmadi."));
    Serial.println(F("    Kontrol et: GND ortak mi? 11V var mi? DATA dogru mu?"));
    return;
  }

  if (adet > 1) {
    Serial.print(F("    DUR: Hatta "));
    Serial.print(adet);
    Serial.println(F(" motor var!"));
    Serial.println(F("    Broadcast ile yazilir - hepsi ayni ID'yi alirdi."));
    Serial.println(F("    ID YAZILMADI. Tek motor birakip tekrar dene."));
    return;
  }

  Serial.print(F("    Tek motor bulundu, mevcut ID: "));
  Serial.println(bulunanIDs[0]);

  if (bulunanIDs[0] == yeniID) {
    Serial.println(F("    Zaten bu ID'de. Islem yapilmadi."));
    return;
  }

  // --- Yaz ---
  Serial.println(F("[2] Yaziliyor..."));
  uint8_t params[2] = { ADDR_ID, yeniID };
  sendPacket(BROADCAST_ID, INST_WRITE, params, 2);
  delay(200);                             // EEPROM yazmasi icin sure

  // --- Dogrula ---
  Serial.println(F("[3] Dogrulama..."));
  if (pingServo(yeniID)) {
    Serial.print(F("    BASARILI - motor artik ID "));
    Serial.println(yeniID);
    Serial.println(F("    Sirakini bagla, yeni ID yaz.\n"));
  } else {
    Serial.println(F("    UYARI: Yeni ID cevap vermiyor. Tekrar tara (s)."));
  }
}

// Bulunan tum motorlari kisaca oynat - hangisi hangisi gormek icin
void hepsiniOynat()
{
  Serial.println(F("\n>>> Dogrulama hareketi"));
  uint8_t adet = tara(true);

  if (adet == 0) {
    Serial.println(F("    Motor yok."));
    return;
  }

  for (uint8_t i = 0; i < adet && i < 32; i++) {
    uint8_t id = bulunanIDs[i];
    Serial.print(F("    ID "));
    Serial.print(id);
    Serial.println(F(" oynuyor"));

    torqueEnable(id, true);
    delay(20);
    setSpeed(id, 150);
    delay(20);

    setPosition(id, 662);
    delay(600);
    setPosition(id, AX12_POS_CENTER);
    delay(600);
  }
  Serial.println(F("    Bitti.\n"));
}

void yardim()
{
  Serial.println(F("\n=== AX-12A ID ARACI ==="));
  Serial.println(F("Komutlar:"));
  Serial.println(F("  <sayi>  1-253: hattaki TEK motora bu ID'yi ata"));
  Serial.println(F("  s       tarama yap"));
  Serial.println(F("  t       bulunan motorlari oynat"));
  Serial.println(F("\n12 motor icin: her seferinde TEK motor bagla,"));
  Serial.println(F("ID'yi yaz, motoru degistir, tekrarla.\n"));
}

void setup()
{
  Serial.begin(115200);
  SERVO_SERIAL.begin(SERVO_BAUD);
  delay(500);

  yardim();

  Serial.println(F("Baslangic taramasi:"));
  tara(false);
  Serial.println(F("\nKomut bekleniyor..."));
}

void loop()
{
  if (!Serial.available()) return;

  String girdi = Serial.readStringUntil('\n');
  girdi.trim();

  if (girdi.length() == 0) return;

  if (girdi == "s" || girdi == "S") {
    Serial.println(F("\n>>> Tarama"));
    tara(false);
    Serial.println();
    return;
  }

  if (girdi == "t" || girdi == "T") {
    hepsiniOynat();
    return;
  }

  // Sayi mi?
  long deger = girdi.toInt();
  if (deger >= 1 && deger <= 253) {
    idAta((uint8_t)deger);
  } else {
    Serial.print(F("Gecersiz girdi: "));
    Serial.println(girdi);
    yardim();
  }
}
