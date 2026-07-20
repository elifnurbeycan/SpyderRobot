#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/InertialUnit.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Config {
    // =========================================================================
    // ARDUINO MEGA PİN VE MOTOR BAĞLANTI YAPILANDIRMASI
    // =========================================================================
    // AX-12A (Coxa ve Tibia eklemleri) -> Serial2 (TX2: 16, RX2: 17) üzerinden half-duplex bağlanır.
    constexpr int AX12_TX_PIN = 16;
    constexpr int AX12_RX_PIN = 17;
    
    // ST3020 (Femur/Uyluk eklemleri)   -> Serial1 (TX1: 18, RX1: 19) üzerinden half-duplex bağlanır.
    constexpr int ST3020_TX_PIN = 18;
    constexpr int ST3020_RX_PIN = 19;
    // HANGİ PİN HANGİ MOTORA BAĞLI? (Haberleşme Hatları Eşleşmesi):
    // - Bacak 1 (Right-Back):   Coxa (AX-12A / Pin 16-17), Femur (ST3020 / Pin 18-19), Tibia (AX-12A / Pin 16-17)
    // - Bacak 2 (Right-Middle): Coxa (AX-12A / Pin 16-17), Femur (ST3020 / Pin 18-19), Tibia (AX-12A / Pin 16-17)
    // - Bacak 3 (Right-Front):  Coxa (AX-12A / Pin 16-17), Femur (ST3020 / Pin 18-19), Tibia (AX-12A / Pin 16-17)
    // - Bacak 4 (Left-Front):   Coxa (AX-12A / Pin 16-17), Femur (ST3020 / Pin 18-19), Tibia (AX-12A / Pin 16-17)
    // - Bacak 5 (Left-Middle):  Coxa (AX-12A / Pin 16-17), Femur (ST3020 / Pin 18-19), Tibia (AX-12A / Pin 16-17)
    // - Bacak 6 (Left-Back):    Coxa (AX-12A / Pin 16-17), Femur (ST3020 / Pin 18-19), Tibia (AX-12A / Pin 16-17)
    // =========================================================================
    // MEKANİK VE CAD BOYUT YAPILANDIRMASI
    // =========================================================================
    // 1. Eklem Uzunlukları (Metre)
    constexpr double L_C = 0.0692; // Coxa (Kalça eklem uzunluğu): 6.92 cm
    constexpr double L_F = 0.0985; // Femur (Uyluk eklem uzunluğu): 9.85 cm
    constexpr double L_T = 0.1515; // Tibia (Kaval/Diz eklem uzunluğu): 15.15 cm
    // 2. Bacaklar Arası Boylamasına ve Enlemesine Gövde Mesafeleri (Metre)
    constexpr double BODY_WIDTH = 0.28594;                  // Sağ ve Sol bacak kalçaları arası toplam genişlik (2 * 14.3 cm)
    constexpr double LEG_SPACING_FRONT_MIDDLE = 0.11159;    // Ön ve Orta bacak arası boylamasına mesafe (11.16 cm)
    constexpr double LEG_SPACING_MIDDLE_BACK = 0.11500;     // Orta ve Arka bacak arası boylamasına mesafe (11.50 cm)

    constexpr int MOTOR_COUNT = 18;
    constexpr double LIMIT = 1.56;

    // ==========================================
    // YÜRÜYÜŞ PARAMETRELERİ (KANIT. ÇALIŞAN)
    // ==========================================
    constexpr double STEP_LENGTH = 0.045;
    constexpr double WALK_SPEED  = 10.0;
    constexpr double STEP_HEIGHT = 0.025;
    constexpr double SWING_PHASE_RATIO = 0.50;

    // Duruş Koordinatları
    constexpr double STANCE_X = 0.230;
    constexpr double STANCE_Y = -0.001;
    constexpr double STANCE_Z = -0.110;

    const double FEMUR_YAW[6] = {
        0.28041, 0.30211, 0.036096, -0.21929, -0.20452, -0.26342
    };

    // Hassas Kalibrasyon Matrisi
    const double HOME_ANGLES[6][3] = {
        { -0.310, -1.357,  1.392 }, // Bacak 1
        { -0.304, -1.232,  0.868 }, // Bacak 2
        { -0.046, -1.189,  1.445 }, // Bacak 3
        {  0.239,  0.831,  0.682 }, // Bacak 4
        {  0.223,  0.981, -1.244 }, // Bacak 5
        {  0.257,  1.036, -1.276 }  // Bacak 6
    };

    const double MOTOR_DIR[6][3] = {
        { 1.0, -1.0,  1.0}, { 1.0, -1.0,  1.0}, { 1.0, -1.0,  1.0},
        {-1.0,  1.0, -1.0}, {-1.0,  1.0, -1.0}, {-1.0,  1.0, -1.0}
    };
}

bool calculateIK(double Px, double Py, double Pz, double &Q0, double &Q1, double &Q2) {
    Q0 = std::atan2(Py, Px);
    double r = std::sqrt(Px * Px + Py * Py);
    double r_prime = r - Config::L_C;
    double z_prime = -Pz;

    double cos_Q2 = (r_prime * r_prime + z_prime * z_prime - Config::L_F * Config::L_F - Config::L_T * Config::L_T) / (2.0 * Config::L_F * Config::L_T);
    cos_Q2 = std::clamp(cos_Q2, -1.0, 1.0);
    double sin_Q2 = -std::sqrt(1.0 - cos_Q2 * cos_Q2);
    Q2 = std::atan2(sin_Q2, cos_Q2);

    double k1 = Config::L_F + Config::L_T * cos_Q2;
    double k2 = Config::L_T * sin_Q2;
    Q1 = std::atan2(k1 * z_prime - k2 * r_prime, k1 * r_prime + k2 * z_prime);
    return true;
}

int main(int argc, char **argv) {
    webots::Robot *robot = new webots::Robot();
    int timeStep = (int)robot->getBasicTimeStep();
    webots::Keyboard keyboard;
    keyboard.enable(timeStep);

    webots::InertialUnit *imu = robot->getInertialUnit("terazi");
    if (imu) {
        imu->enable(timeStep);
    }

    std::vector<webots::Motor*> motors(Config::MOTOR_COUNT);
    for (int i = 0; i < Config::MOTOR_COUNT; i++) {
        motors[i] = robot->getMotor("joint_" + std::to_string(i + 1));
        if (motors[i]) motors[i]->setVelocity(6.0);
    }

    double OFFSET[6][3] = {0.0};
    for (int leg = 0; leg < 6; leg++) {
        double yaw = Config::FEMUR_YAW[leg];
        double local_X = Config::STANCE_X * std::cos(-yaw) - Config::STANCE_Y * std::sin(-yaw);
        double local_Y = Config::STANCE_X * std::sin(-yaw) + Config::STANCE_Y * std::cos(-yaw);
        double local_Z = Config::STANCE_Z;

        double q0, q1, q2;
        calculateIK(local_X, local_Y, local_Z, q0, q1, q2);

        OFFSET[leg][0] = Config::HOME_ANGLES[leg][0] - (q0 * Config::MOTOR_DIR[leg][0]);
        OFFSET[leg][1] = Config::HOME_ANGLES[leg][1] - (q1 * Config::MOTOR_DIR[leg][1]);
        OFFSET[leg][2] = Config::HOME_ANGLES[leg][2] - (q2 * Config::MOTOR_DIR[leg][2]);
    }

    double t = 0.0;
    bool isWalking = false;

    // Uzaktan kumanda / klavye durum değişkenleri
    double vy_current = 0.0;
    double omega_current = 0.0;
    double walk_scale = 0.0;
    double target_yaw = 0.0;
    bool wasWalking = false;

    while (robot->step(timeStep) != -1) {
        int key = keyboard.getKey();

        // Klavye komutlarını oku
        double vy_target = 0.0;
        double omega_target = 0.0;
        bool hasInput = false;

        if (key == webots::Keyboard::UP || key == 'W') {
            vy_target = 1.0;
            hasInput = true;
        } else if (key == webots::Keyboard::DOWN || key == 'S') {
            vy_target = -1.0;
            hasInput = true;
        } else if (key == webots::Keyboard::LEFT || key == 'A') {
            omega_target = 1.0;
            hasInput = true;
        } else if (key == webots::Keyboard::RIGHT || key == 'D') {
            omega_target = -1.0;
            hasInput = true;
        }

        // Yumuşak geçiş filtresi
        double accel_rate = 0.15;
        vy_current += (vy_target - vy_current) * accel_rate;
        omega_current += (omega_target - omega_current) * accel_rate;

        if (hasInput) {
            walk_scale = std::min(walk_scale + accel_rate, 1.0);
        } else {
            walk_scale = std::max(walk_scale - accel_rate, 0.0);
        }

        isWalking = (walk_scale > 0.01);
        if (isWalking) {
            t += (timeStep / 1000.0) * Config::WALK_SPEED;
        }

        // IMU yön koruma
        double yaw_current = 0.0;
        if (imu) {
            const double *rpy = imu->getRollPitchYaw();
            yaw_current = rpy[2];
        }

        if (omega_target != 0.0 || !isWalking) {
            target_yaw = yaw_current;
            wasWalking = isWalking;
        } else if (isWalking && !wasWalking) {
            target_yaw = yaw_current;
            wasWalking = true;
        }

        double yaw_error = target_yaw - yaw_current;
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        double Kp = 0.15;
        double steer_correction = (omega_target == 0.0 && vy_target != 0.0) ? (Kp * yaw_error) : 0.0;
        // Düzeltmeyi küçük bir aralıkla sınırla — ayak pozisyonlarını bozmamak için
        steer_correction = std::clamp(steer_correction, -0.005, 0.005);

        for (int leg = 0; leg < 6; leg++) {
            bool isGroup1 = (leg == 0 || leg == 4 || leg == 2);
            double phase = isGroup1 ? t : t + M_PI;
            phase = std::fmod(phase, 2.0 * M_PI);

            double phase_norm = phase / (2.0 * M_PI);

            double dY = 0.0, dZ = 0.0;
            if (isWalking) {
                double side_sign = (leg < 3) ? 1.0 : -1.0;
                double leg_vy = vy_current + side_sign * omega_current;

                if (phase_norm < Config::SWING_PHASE_RATIO) {
                    // SWING (Havada atılım) — Orijinal sin() formülü
                    double swing_p = phase_norm / Config::SWING_PHASE_RATIO;
                    dY = -Config::STEP_LENGTH * std::cos(swing_p * M_PI) * leg_vy;
                    dZ = Config::STEP_HEIGHT * std::sin(swing_p * M_PI);
                } else {
                    // STANCE (Yerde itiş)
                    double stance_p = (phase_norm - Config::SWING_PHASE_RATIO) / (1.0 - Config::SWING_PHASE_RATIO);
                    dY = Config::STEP_LENGTH * std::cos(stance_p * M_PI) * leg_vy;
                    dZ = 0.0;
                }

                dY *= walk_scale;
                dZ *= walk_scale;
            }

            double target_X = Config::STANCE_X;
            double side_sign = (leg < 3) ? 1.0 : -1.0;
            double target_Y = Config::STANCE_Y + dY + side_sign * steer_correction;
            double target_Z = Config::STANCE_Z + dZ;

            double yaw = Config::FEMUR_YAW[leg];
            double local_X = target_X * std::cos(-yaw) - target_Y * std::sin(-yaw);
            double local_Y = target_X * std::sin(-yaw) + target_Y * std::cos(-yaw);
            double local_Z = target_Z;

            double q0, q1, q2;
            if (calculateIK(local_X, local_Y, local_Z, q0, q1, q2)) {
                double m0 = std::clamp((q0 * Config::MOTOR_DIR[leg][0]) + OFFSET[leg][0], -Config::LIMIT, Config::LIMIT);
                double m1 = std::clamp((q1 * Config::MOTOR_DIR[leg][1]) + OFFSET[leg][1], -Config::LIMIT, Config::LIMIT);
                double m2 = std::clamp((q2 * Config::MOTOR_DIR[leg][2]) + OFFSET[leg][2], -Config::LIMIT, Config::LIMIT);

                motors[leg * 3 + 0]->setPosition(m0);
                motors[leg * 3 + 1]->setPosition(m1);
                motors[leg * 3 + 2]->setPosition(m2);
            }
        }
    }
    delete robot;
    return 0;
}
