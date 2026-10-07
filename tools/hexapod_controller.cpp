/*
 * hexapod_controller.cpp  –  Webots simülasyonunun gerçek donanıma uyarlaması
 *
 * Webots'un kaldırılan kısımları:
 *   webots::Robot, Motor, Keyboard, InertialUnit  →  POSIX serial + termios klavye
 *
 * Donanım bağlantısı:
 *   Arduino Mega (/dev/ttyUSBx, 115200 baud) ← mega_full_bridge.cpp firmware
 *   Arduino → ST3020 (Serial1 18/19)
 *   Arduino → AX-12A (Serial2 16/17)
 *
 * Derleme:
 *   g++ -std=c++17 -O2 -o hexapod hexapod_controller.cpp && ./hexapod /dev/ttyUSB1
 *
 * Klavye:
 *   W/↑  ileri    S/↓  geri    A/←  sol    D/→  sağ    Q  çıkış
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>

// POSIX serial
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/time.h>
#include <cstring>
#include <cerrno>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// YAPILANDIRMA  –  Simülasyonla aynı mekanik parametreler
// ============================================================================
namespace Config {
    constexpr double L_C = 0.0692;
    constexpr double L_F = 0.0985;
    constexpr double L_T = 0.1515;

    constexpr double BODY_WIDTH              = 0.28594;
    constexpr double LEG_SPACING_FRONT_MIDDLE = 0.11159;
    constexpr double LEG_SPACING_MIDDLE_BACK  = 0.11500;

    constexpr double LIMIT            = 1.56;
    constexpr double STEP_LENGTH      = 0.045;
    constexpr double WALK_SPEED       = 10.0;
    constexpr double STEP_HEIGHT      = 0.025;
    constexpr double SWING_PHASE_RATIO = 0.50;

    constexpr double STANCE_X = 0.230;
    constexpr double STANCE_Y = -0.001;
    constexpr double STANCE_Z = -0.110;

    // Femur yaw (simülasyondan)
    const double FEMUR_YAW[6] = {
        0.28041, 0.30211, 0.036096, -0.21929, -0.20452, -0.26342
    };

    // Motor açısı stance konumunda (radyan, Webots çerçevesinde)
    // Encoder dönüşüm formülü: enc = STANCE_ENC + (motorAci - HOME_ANGLES) * CPR * ENC_SIGN
    const double HOME_ANGLES[6][3] = {
        { -0.310, -1.357,  1.392 },  // Bacak 0 – Right-Back
        { -0.304, -1.232,  0.868 },  // Bacak 1 – Right-Middle
        { -0.046, -1.189,  1.445 },  // Bacak 2 – Right-Front
        {  0.239,  0.831,  0.682 },  // Bacak 3 – Left-Front
        {  0.223,  0.981, -1.244 },  // Bacak 4 – Left-Middle
        {  0.257,  1.036, -1.276 }   // Bacak 5 – Left-Back
    };

    const double MOTOR_DIR[6][3] = {
        { 1.0, -1.0,  1.0}, { 1.0, -1.0,  1.0}, { 1.0, -1.0,  1.0},
        {-1.0,  1.0, -1.0}, {-1.0, -1.0, -1.0}, {-1.0,  1.0, -1.0}  // Leg4 femur: sağ taraf fiziksel montaj
    };
}

// ============================================================================
// MOTOR KALİBRASYON TABLOSU
//
//  Sıra: [leg][joint]  →  joint 0=coxa, 1=femur, 2=tibia
//  Bacak düzeni: 0=R-Back 1=R-Mid 2=R-Front 3=L-Front 4=L-Mid 5=L-Back
//
//  MOTOR_IDS : servo ID'si (mega_full_bridge bunu doğru porta yönlendirir)
//  STANCE_ENC: home_position.py'daki ST3020_STANCE / AX12_STANCE değerleri
//  CPR        : sayaç/radyan  (ST3020=651.9, AX-12A=195.4)
//  ENC_SIGN   : +1 veya -1  — fiziksel yön kalibrasyonu
//               BAŞLANGITA HEPSI +1, test ederek çevir
// ============================================================================
struct MotorCal {
    uint8_t  id;          // Servo ID
    double   stance_enc;  // Encoder değeri stance konumunda
    double   cpr;         // Sayaç / radyan
    double   enc_sign;    // +1 veya -1
};

// Femur ST3020 mekanik limitleri (femur_limits.py, ölçüm 2026-07-20)
//   Sıra leg 0..5 = ID 2,5,8,11,14,17.  {min_enc, max_enc}
// IK saçma bir açı üretirse femuru mekanik aralık DIŞINA zorlamamak için
// angleToEnc bu değerlere clamp eder.
const int FEMUR_ENC_LIMIT[6][2] = {
    {3046, 4090},  // leg0  ID2   Femur 1
    {1487, 3395},  // leg1  ID5   Femur 2
    {3220, 4091},  // leg2  ID8   Femur 3
    {2384, 4091},  // leg3  ID11  Femur 4
    {3362, 4091},  // leg4  ID14  Femur 5  ← zorlanma sorunu buradaydı
    {2303, 4090},  // leg5  ID17  Femur 6
};

//                  ID   stance_enc   cpr      enc_sign
MotorCal CAL[6][3] = {
    // Bacak 0 – Right-Back   (coxa=ID3 AX, femur=ID2 ST, tibia=ID1 AX)
    {{ 3,  563.0, 195.4, +1.0},   // coxa (ID 3)
     { 2, 3283.0, 651.9, +1.0},   // femur (ID 2)
     { 1,  465.0, 195.4, +1.0}},  // tibia (ID 1)
    // Bacak 1 – Right-Middle (coxa=ID6 AX, femur=ID5 ST, tibia=ID4 AX)
    {{ 6,  527.0, 195.4, +1.0},   // coxa (ID 6)
     { 5, 1918.0, 651.9, +1.0},   // femur (ID 5)
     { 4,  699.0, 195.4, +1.0}},  // tibia (ID 4)
    // Bacak 2 – Right-Front  (coxa=ID9 AX, femur=ID8 ST, tibia=ID7 AX)
    {{ 9,  518.0, 195.4, +1.0},   // coxa (ID 9)
     { 8, 3616.0, 651.9, -1.0},   // femur (ID 8)
     { 7,  403.0, 195.4, +1.0}},  // tibia (ID 7)
    // Bacak 3 – Left-Front   (coxa=ID10 AX, femur=ID11 ST, tibia=ID12 AX)
    {{10,  821.0, 195.4, +1.0},
     {11, 2997.0, 651.9, +1.0},
     {12,  593.0, 195.4, +1.0}},
    // Bacak 4 – Left-Middle  (coxa=ID13 AX, femur=ID14 ST, tibia=ID15 AX)
    {{13, 1023.0, 195.4, +1.0},
     {14, 3716.0, 651.9, +1.0},   // 3716 (yeniden kalibre)
     {15,  908.0, 195.4, +1.0}},
    // Bacak 5 – Left-Back    (coxa=ID16 AX, femur=ID17 ST, tibia=ID18 AX)
    {{16,  496.0, 195.4, +1.0},
     {17, 2994.0, 651.9, +1.0},
     {18,  303.0, 195.4, +1.0}},
};

// Motor açısı (radyan) → encoder pozisyonu
int angleToEnc(int leg, int joint, double motorAngle) {
    const MotorCal &c = CAL[leg][joint];
    double raw = c.stance_enc
                + (motorAngle - Config::HOME_ANGLES[leg][joint]) * c.cpr * c.enc_sign;
    // Sınırla: femur (joint 1) için ölçülmüş mekanik limit, diğerleri sayaç aralığı
    double lo, hi;
    if (joint == 1) {          // femur = ST3020
        lo = FEMUR_ENC_LIMIT[leg][0];
        hi = FEMUR_ENC_LIMIT[leg][1];
    } else {                   // coxa / tibia = AX-12A
        lo = 0.0;
        hi = 1023.0;
    }
    return (int)std::round(std::clamp(raw, lo, hi));
}

// ============================================================================
// SERİ PORT
// ============================================================================
int g_fd = -1;

void serialWrite(const std::string &s) {
    write(g_fd, s.c_str(), s.size());
}

void sendMotor(int leg, int joint, double motorAngle) {
    int enc = angleToEnc(leg, joint, motorAngle);
    uint8_t id = CAL[leg][joint].id;
    std::string cmd = "M " + std::to_string(id) + " " + std::to_string(enc) + "\n";
    serialWrite(cmd);
    usleep(500);   // 0.5 ms gecikme
}

int openSerial(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return -1; }
    struct termios t{};
    tcgetattr(fd, &t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    cfmakeraw(&t);
    t.c_cc[VMIN]  = 0;
    t.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &t);
    return fd;
}

// ============================================================================
// KLAVYE  –  raw termios, non-blocking
// ============================================================================
struct Keyboard {
    struct termios orig{};
    bool is_tty = false;
    void init() {
        is_tty = isatty(STDIN_FILENO);
        if (!is_tty) return;
        tcgetattr(STDIN_FILENO, &orig);
        struct termios raw = orig;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    void restore() {
        if (is_tty) tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    }
    // Dönen değer: 'W' 'S' 'A' 'D' 'Q' veya 0
    int read() {
        if (!is_tty) return 0;
        char buf[4] = {0};
        int n = ::read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0) return 0;
        // ESC dizisi (ok tuşu): ESC [ A/B/C/D
        if (n >= 3 && buf[0] == 27 && buf[1] == '[') {
            switch (buf[2]) {
                case 'A': return 'W';
                case 'B': return 'S';
                case 'C': return 'D';
                case 'D': return 'A';
            }
        }
        char c = toupper(buf[0]);
        if (c == 'W' || c == 'S' || c == 'A' || c == 'D' || c == 'Q') return c;
        return 0;
    }
};

// ============================================================================
// TERS KİNEMATİK  –  simülasyonla aynı
// ============================================================================
bool calculateIK(double Px, double Py, double Pz,
                 double &Q0, double &Q1, double &Q2) {
    Q0 = std::atan2(Py, Px);
    double r       = std::sqrt(Px*Px + Py*Py);
    double r_prime = r - Config::L_C;
    double z_prime = -Pz;

    double cos_Q2 = (r_prime*r_prime + z_prime*z_prime
                     - Config::L_F*Config::L_F - Config::L_T*Config::L_T)
                    / (2.0 * Config::L_F * Config::L_T);
    cos_Q2 = std::clamp(cos_Q2, -1.0, 1.0);
    double sin_Q2 = -std::sqrt(1.0 - cos_Q2*cos_Q2);
    Q2 = std::atan2(sin_Q2, cos_Q2);

    double k1 = Config::L_F + Config::L_T * cos_Q2;
    double k2 = Config::L_T * sin_Q2;
    Q1 = std::atan2(k1*z_prime - k2*r_prime,
                    k1*r_prime + k2*z_prime);
    return true;
}

// ============================================================================
// ZAMAN YARDIMCISI
// ============================================================================
double now_sec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// ============================================================================
// ANA DÖNGÜ
// ============================================================================
int main(int argc, char **argv) {
    const char *dev = "/dev/ttyUSB0";
    bool auto_walk = false;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--auto" || arg == "-a" || arg == "auto") auto_walk = true;
        else if (arg.rfind("/dev/", 0) == 0) dev = argv[i];
    }

    g_fd = openSerial(dev);
    if (g_fd < 0) return 1;
    std::cout << "Seri port açık: " << dev << std::endl;
    sleep(2);  // Arduino reset bekle
    tcflush(g_fd, TCIOFLUSH);

    Keyboard kb;
    kb.init();
    if (!kb.is_tty) auto_walk = true;

    if (auto_walk) {
        std::cout << "Otomatik Yürüme Modu aktif. Çıkmak için Ctrl+C." << std::endl;
    } else {
        std::cout << "Başladı. W/S/A/D ile hareket, Q ile çıkış." << std::endl;
    }

    // ── OFFSET hesapla (simülasyonla birebir aynı) ──────────────────────────
    double OFFSET[6][3] = {};
    for (int leg = 0; leg < 6; leg++) {
        double yaw    = Config::FEMUR_YAW[leg];
        double lX     = Config::STANCE_X * std::cos(-yaw) - Config::STANCE_Y * std::sin(-yaw);
        double lY     = Config::STANCE_X * std::sin(-yaw) + Config::STANCE_Y * std::cos(-yaw);
        double q0, q1, q2;
        calculateIK(lX, lY, Config::STANCE_Z, q0, q1, q2);
        OFFSET[leg][0] = Config::HOME_ANGLES[leg][0] - q0 * Config::MOTOR_DIR[leg][0];
        OFFSET[leg][1] = Config::HOME_ANGLES[leg][1] - q1 * Config::MOTOR_DIR[leg][1];
        OFFSET[leg][2] = Config::HOME_ANGLES[leg][2] - q2 * Config::MOTOR_DIR[leg][2];
    }

    // ── Durum değişkenleri ──────────────────────────────────────────────────
    const int TIME_STEP_MS = 20;   // 50 Hz
    double t             = 0.0;
    double vy_current    = 0.0;
    double omega_current = 0.0;
    double walk_scale    = 0.0;
    const double accel_rate = 0.15;

    double loop_start = now_sec();

    while (true) {
        int key = kb.read();
        if (key == 'Q') break;

        double vy_target    = 0.0;
        double omega_target = 0.0;
        bool   hasInput     = false;

        if (auto_walk) {
            vy_target = 1.0;
            hasInput  = true;
        } else {
            if (key == 'W') { vy_target =  1.0; hasInput = true; }
            if (key == 'S') { vy_target = -1.0; hasInput = true; }
            if (key == 'A') { omega_target =  1.0; hasInput = true; }
            if (key == 'D') { omega_target = -1.0; hasInput = true; }
        }

        vy_current    += (vy_target    - vy_current)    * accel_rate;
        omega_current += (omega_target - omega_current) * accel_rate;
        walk_scale = hasInput
                   ? std::min(walk_scale + accel_rate, 1.0)
                   : std::max(walk_scale - accel_rate, 0.0);

        bool isWalking = (walk_scale > 0.01);
        if (isWalking)
            t += (TIME_STEP_MS / 1000.0) * Config::WALK_SPEED;

        // ── Her bacak için IK + motor komutu ────────────────────────────────
        for (int leg = 0; leg < 6; leg++) {
            bool isGroup1  = (leg == 0 || leg == 2 || leg == 4);
            double phase   = isGroup1 ? t : t + M_PI;
            phase          = std::fmod(phase, 2.0 * M_PI);
            double pn      = phase / (2.0 * M_PI);

            double dY = 0.0, dZ = 0.0;
            if (isWalking) {
                double side_sign = (leg < 3) ? 1.0 : -1.0;
                double leg_vy    = vy_current + side_sign * omega_current;

                if (pn < Config::SWING_PHASE_RATIO) {
                    double sp = pn / Config::SWING_PHASE_RATIO;
                    dY = -Config::STEP_LENGTH * std::cos(sp * M_PI) * leg_vy;
                    dZ =  Config::STEP_HEIGHT * std::sin(sp * M_PI);
                } else {
                    double sp = (pn - Config::SWING_PHASE_RATIO)
                                / (1.0 - Config::SWING_PHASE_RATIO);
                    dY = Config::STEP_LENGTH * std::cos(sp * M_PI) * leg_vy;
                    dZ = 0.0;
                }
                dY *= walk_scale;
                dZ *= walk_scale;
            }

            double target_X = Config::STANCE_X;
            double side_sign = (leg < 3) ? 1.0 : -1.0;
            double target_Y  = Config::STANCE_Y + dY;
            double target_Z  = Config::STANCE_Z + dZ;

            double yaw  = Config::FEMUR_YAW[leg];
            double lX   = target_X * std::cos(-yaw) - target_Y * std::sin(-yaw);
            double lY   = target_X * std::sin(-yaw) + target_Y * std::cos(-yaw);

            double q0, q1, q2;
            if (calculateIK(lX, lY, target_Z, q0, q1, q2)) {
                double m0 = std::clamp(q0 * Config::MOTOR_DIR[leg][0] + OFFSET[leg][0],
                                       -Config::LIMIT, Config::LIMIT);
                double m1 = std::clamp(q1 * Config::MOTOR_DIR[leg][1] + OFFSET[leg][1],
                                       -Config::LIMIT, Config::LIMIT);
                double m2 = std::clamp(q2 * Config::MOTOR_DIR[leg][2] + OFFSET[leg][2],
                                       -Config::LIMIT, Config::LIMIT);
                sendMotor(leg, 0, m0);
                sendMotor(leg, 1, m1);
                sendMotor(leg, 2, m2);
            }
        }

        // ── 50 Hz döngü zamanlaması ─────────────────────────────────────────
        double elapsed = (now_sec() - loop_start) * 1000.0;
        int    remain  = TIME_STEP_MS - (int)elapsed;
        if (remain > 0) usleep(remain * 1000);
        loop_start = now_sec();
    }

    kb.restore();
    close(g_fd);
    std::cout << "Çıkış." << std::endl;
    return 0;
}
