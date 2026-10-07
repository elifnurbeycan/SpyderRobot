/*
 * motor_calib.h - 18 motor mekanik limit / merkez kalibrasyonu
 *
 * Web arayuzu (tools/motor_ui/app_all.py) ile ELLE olculdu:
 * her motor serbest birakilip min ve max uclarina cevrildi, o an okunan
 * ham pozisyon (present position) kaydedildi.
 *
 * SARMA (wrap): ST3020 ve AX-12A pozisyon sayaci N'de (ST:4096, AX:1024)
 * basa doner. Bir eklemin calisma bolgesi sarma noktasinin ustundeyse
 * min > max ya da fark cok kucuk gorunur. wrap=1 olan motorlarda gercek
 * hareket icin max'i (max + N) olarak yorumla. Merkez zaten dogru hesaplandi.
 *
 * Alanlar: id, min_pos, max_pos, merkez (center), wrap (0/1)
 * Tip: motor_ids.h -> FEMUR=ST3020 (N=4096), COXA/TIBIA=AX-12A (N=1024)
 *
 * Kaynak eslesmesi: [[hexapod-motor-mapping]] / motor_ids.h
 */

#ifndef MOTOR_CALIB_H
#define MOTOR_CALIB_H

#include <Arduino.h>

#define ST_N   4096      // ST3020 sayac araligi
#define AX_N   1024      // AX-12A sayac araligi

struct MotorCalib {
  uint8_t  id;
  uint16_t min_pos;      // olculen alt uc (ham)
  uint16_t max_pos;      // olculen ust uc (ham; wrap ise sarma sonrasi kucuk deger)
  uint16_t center;       // hesaplanan gercek merkez (0..N-1)
  uint8_t  wrap;         // 1 = calisma bolgesi sarma noktasini geciyor
};

// 18 motor. Yorumdaki derece = olculen toplam hareket araligi.
const MotorCalib MOTOR_CALIB[18] = {
  { 1,   47,  934,  491, 0}, // B1 Tibia  AX-12A  259.9 deg
  { 2, 2871,  877, 3922, 1}, // B1 Femur  ST3020  184.7 deg
  { 3,  224,  802,  513, 0}, // B1 Coxa   AX-12A  169.3 deg
  { 4,  356,    0,  690, 1}, // B2 Tibia  AX-12A  195.7 deg
  { 5, 1370, 3477, 2424, 0}, // B2 Femur  ST3020  185.2 deg
  { 6,  833,  189, 1023, 1}, // B2 Coxa   AX-12A  111.3 deg
  { 7,   52,  896,  474, 0}, // B3 Tibia  AX-12A  247.3 deg
  { 8, 3133, 1167,  102, 1}, // B3 Femur  ST3020  187.2 deg
  { 9,  832,  186, 1021, 1}, // B3 Coxa   AX-12A  110.7 deg
  {10,  494,  541,    6, 1}, // B4 Coxa   AX-12A  313.8 deg
  {11, 2450,  560, 3553, 1}, // B4 Femur  ST3020  193.9 deg
  {12,  971,   84,   16, 1}, // B4 Tibia  AX-12A   40.1 deg
  {13,  743,  158,  963, 1}, // B5 Coxa   AX-12A  128.6 deg
  {14, 3263, 1121,  144, 1}, // B5 Femur  ST3020  171.7 deg
  {15,   51,  399,  225, 0}, // B5 Tibia  AX-12A  102.0 deg
  {16,  188,  827,  508, 0}, // B6 Coxa   AX-12A  187.2 deg
  {17, 2357,  421, 3437, 1}, // B6 Femur  ST3020  189.8 deg
  {18,  661, 1004,  833, 0}, // B6 Tibia  AX-12A  100.5 deg
};

// id -> kalibrasyon kaydi (bulunamazsa NULL)
inline const MotorCalib* calibBul(uint8_t id) {
  for (uint8_t i = 0; i < 18; i++)
    if (MOTOR_CALIB[i].id == id) return &MOTOR_CALIB[i];
  return 0;
}

#endif // MOTOR_CALIB_H
