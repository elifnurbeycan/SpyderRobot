/*
 * ST3020 interaktif ID atama araci - Arduino Mega 2560
 *
 * AX-12A ARACINDAN FARKI:
 *   - ID adresi 5 (AX-12A'da 3)
 *   - EEPROM kilidi var: yazmadan once acilmali, sonra kapatilmali
 *   - Pozisyon araligi 0..4095 (AX-12A'da 0..1023)
 *   - Checksum ve baudrate ayni
 *
 * BIR KEZ YUKLE, MOTORLARI SIRAYLA GEC.
 *
 * KULLANIM:
 *   1. Hatta SADECE bir ST3020 bagla
 *   2. Monitore atamak istedigin ID'yi yaz + Enter
 *   3. Motoru cikar, sirakini bagla, yeni ID yaz
 *
 * KOMUTLAR:
 *   <sayi>   1-253 arasi: hattaki tek motora bu ID'yi ata
 *   s        tarama yap
 *   t        bulunan motorlari kisaca oynat
 *
 * BAGLANTI:
 *   Mega TX1 (pin 18) --[1k]--+--- ST3020 DATA
 *   Mega RX1 (pin 19) --------+
 *   Mega GND ----------------------- ST3020 GND ---- Guc (-)
 *   Guc (+) 11V -------------------- ST3020 VCC
 *
 * NOT: ID atarken tek motor bagli oldugu icin TX1/RX1 kullaniliyor.
 * Calisma sirasinda ST3020'ler ayri hatta (TX2/RX2) baglanacak.
 */

#include <Arduino.h>

#define SERVO_SERIAL Serial1
#define SERVO_BAUD   1000000UL

#define HEADER1        0xFF
#define HEADER2        0xFF
#define INST_PING      0x01
#define INST_WRITE     0x03
#define BROADCAST_ID   0xFE     // 254 = hattaki herkes

// ST3020 (Feetech STS) kontrol tablosu adresleri - AX-12A'DAN FARKLI
#define ADDR_ID              5      // AX-12A'da 3
#define ADDR_LOCK            55     // EEPROM kilidi: 0 = acik, 1 = kilitli
#define ADDR_TORQUE_ENABLE   40     // AX-12A'da 24
#define ADDR_GOAL_POSITION   42     // AX-12A'da 30

#define STS_POS_MAX      4095       // AX-12A'da 1023
#define STS_POS_CENTER   2048

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

  // Kendi gonderdigimiz 6 byte half-duplex hatta geri okunur.
  return (bytesIn > 6);
}

uint8_t tara(bool sessiz)
{
  bulunanSayi = 0;

  for (uint16_t id = 0; id < 254; id++) {
    if (pingServo((uint8_t)id)) {
      if (bulunanSayi < 32) bulunanIDs[bulunanSayi] = (uint8_t)id;
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

// ---------------------------------------------------------------
// EEPROM kilidi. ST3020'de ID yazmadan once acilmali.
// AX-12A'da boyle bir sey yok - bu araca ozel.
// ---------------------------------------------------------------
void eepromLock(uint8_t id, bool kilitle)
{
  uint8_t params[2] = { ADDR_LOCK, (uint8_t)(kilitle ? 1 : 0) };
  sendPacket(id, INST_WRITE, params, 2);
  delay(50);
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

// ---------------------------------------------------------------
// ID atama. Hatta tek motor olmasi sart.
// ---------------------------------------------------------------
void idAta(uint8_t yeniID)
{
  Serial.print(F("\n>>> ID "));
  Serial.print(yeniID);
  Serial.println(F(" atanacak (ST3020)"));

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

  uint8_t mevcutID = bulunanIDs[0];
  Serial.print(F("    Tek motor bulundu, mevcut ID: "));
  Serial.println(mevcutID);

  if (mevcutID == yeniID) {
    Serial.println(F("    Zaten bu ID'de. Islem yapilmadi."));
    return;
  }

  // --- EEPROM kilidini ac ---
  Serial.println(F("[2] EEPROM kilidi aciliyor..."));
  eepromLock(mevcutID, false);

  // --- ID yaz ---
  Serial.println(F("[3] ID yaziliyor..."));
  uint8_t params[2] = { ADDR_ID, yeniID };
  sendPacket(mevcutID, INST_WRITE, params, 2);
  delay(200);                             // EEPROM yazmasi icin sure

  // --- Kilidi geri kapat (yeni ID uzerinden) ---
  Serial.println(F("[4] EEPROM kilidi kapatiliyor..."));
  eepromLock(yeniID, true);

  // --- Dogrula ---
  Serial.println(F("[5] Dogrulama..."));
  if (pingServo(yeniID)) {
    Serial.print(F("    BASARILI - motor artik ID "));
    Serial.println(yeniID);
    Serial.println(F("    Sirakini bagla, yeni ID yaz.\n"));
  } else {
    Serial.println(F("    UYARI: Yeni ID cevap vermiyor. Tekrar tara (s)."));
  }
}

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

    setPosition(id, 2648, 1000);
    delay(600);
    setPosition(id, STS_POS_CENTER, 1000);
    delay(600);
  }
  Serial.println(F("    Bitti.\n"));
}

void yardim()
{
  Serial.println(F("\n=== ST3020 ID ARACI ==="));
  Serial.println(F("Komutlar:"));
  Serial.println(F("  <sayi>  1-253: hattaki TEK motora bu ID'yi ata"));
  Serial.println(F("  s       tarama yap"));
  Serial.println(F("  t       bulunan motorlari oynat"));
  Serial.println(F("\nHer seferinde TEK motor bagla, ID'yi yaz, degistir.\n"));
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

  long deger = girdi.toInt();
  if (deger >= 1 && deger <= 253) {
    idAta((uint8_t)deger);
  } else {
    Serial.print(F("Gecersiz girdi: "));
    Serial.println(girdi);
    yardim();
  }
}
