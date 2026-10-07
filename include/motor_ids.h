/*
 * motor_ids.h - Hexapod motor ID eslemesi (tek kaynak)
 *
 * 6 bacak x 3 eklem = 18 motor, ID araligi 1-18 (bosluk yok).
 *
 * IKI FARKLI MOTOR TIPI, IKI AYRI SERI HAT:
 *
 *   FEMUR  -> ST3020 (Feetech STS)  : Serial1 (TX1=18, RX1=19) @ 1 Mbaud
 *   COXA   -> AX-12A (Dynamixel)    : Serial2 (TX2=16, RX2=17) @ 1 Mbaud
 *   TIBIA  -> AX-12A (Dynamixel)    : Serial2 (TX2=16, RX2=17) @ 1 Mbaud
 *
 * Yani sadece femurlar ST3020, coxa ve tibia'larin hepsi AX-12A.
 * Iki tip FARKLI PROTOKOL konusur - ayni fonksiyonla surulemezler.
 *
 * !!! DIKKAT: EKLEM SIRASI IKI BLOKTA TERS !!!
 *   Bacak 1-3 : tibia -> femur -> coxa  (ornek bacak 1: 1, 2, 3)
 *   Bacak 4-6 : coxa  -> femur -> tibia (ornek bacak 4: 10, 11, 12)
 * Femurlar her iki blokta da ortada kalir. Dizileri elle siralamaya
 * calisma - asagidaki tablolari kullan.
 *
 * Dizi indeksi 0-5 = bacak 1-6.
 */

#ifndef MOTOR_IDS_H
#define MOTOR_IDS_H

#include <Arduino.h>

#define BACAK_SAYISI 6

// --- ST3020 (Feetech STS) - Serial1 @ 1 Mbaud ---
#define FEMUR_SERIAL  Serial1
#define FEMUR_BAUD    1000000UL

const uint8_t FEMUR_IDS[BACAK_SAYISI] = { 2, 5, 8, 11, 14, 17 };

// --- AX-12A (Dynamixel) - Serial2 @ 1 Mbaud ---
#define AX_SERIAL     Serial2
#define AX_BAUD       1000000UL

const uint8_t COXA_IDS[BACAK_SAYISI]  = { 3, 6, 9, 10, 13, 16 };
const uint8_t TIBIA_IDS[BACAK_SAYISI] = { 1, 4, 7, 12, 15, 18 };

#endif // MOTOR_IDS_H
