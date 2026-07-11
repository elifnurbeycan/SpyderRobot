#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <algorithm>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace Config {
    constexpr double L_C = 0.0692; // Fiziksel CAD ölçümü: 6.92 cm
    constexpr double L_F = 0.0985; // Fiziksel CAD ölçümü: 9.85 cm
    constexpr double L_T = 0.1515; // Fiziksel CAD ölçümü: 15.15 cm
    constexpr int MOTOR_COUNT = 18;
    constexpr double LIMIT = 1.56;

    // Koşu Parametreleri
    constexpr double STEP_LENGTH = 0.050;  
    constexpr double STEP_HEIGHT = 0.025; // Adımın havada 2.5 cm yükselmesini sağladık (sürtünmeyi önlemek için)
    constexpr double WALK_SPEED  = 4.5;   
    constexpr double SWING_PHASE_RATIO = 0.5; // Adımın %50'si havada (atılım), %50'si yerde (itiş)

    // Standart Duruş Koordinatları
    constexpr double STANCE_X = 0.230;
    constexpr double STANCE_Y = -0.001;
    constexpr double STANCE_Z = -0.110; // Eklemlerin limitlerini aşmaması için ideal yükseklik (-11 cm)

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
    int prevKey = -1;

    while (robot->step(timeStep) != -1) {
        int key = keyboard.getKey();
        if (key == ' ' && prevKey != ' ') {
            isWalking = !isWalking;
            std::cout << (isWalking ? ">> KOSU BASLADI" : ">> DURDU") << std::endl;
        }
        prevKey = key;

        if (isWalking) t += (timeStep / 1000.0) * Config::WALK_SPEED;

        for (int leg = 0; leg < 6; leg++) {
            bool isGroup1 = (leg == 0 || leg == 4 || leg == 2);
            double phase = isGroup1 ? t : t + M_PI;
            phase = std::fmod(phase, 2.0 * M_PI);
            
            // Fazı 0.0 ile 1.0 arasına normalize et
            double phase_norm = phase / (2.0 * M_PI);

            double dY = 0.0, dZ = 0.0;
            if (isWalking) {
                if (phase_norm < Config::SWING_PHASE_RATIO) {
                    // SWING (Havada atılım)
                    double swing_p = phase_norm / Config::SWING_PHASE_RATIO;
                    dY = -Config::STEP_LENGTH * std::cos(swing_p * M_PI); 
                    dZ = Config::STEP_HEIGHT * std::sin(swing_p * M_PI);  
                } else {
                    // STANCE (Yerde itiş)
                    double stance_p = (phase_norm - Config::SWING_PHASE_RATIO) / (1.0 - Config::SWING_PHASE_RATIO);
                    dY = Config::STEP_LENGTH * std::cos(stance_p * M_PI); 
                    dZ = 0.0; 
                }
            }

            double target_X = Config::STANCE_X;
            double target_Y = Config::STANCE_Y + dY;
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

                if (isWalking) {
                    static std::ofstream logFile("/home/elif/Downloads/robolig_sim/controller_log.txt");
                    if (logFile.is_open()) {
                        logFile << "t=" << t << " leg=" << leg
                                << " target=(" << target_X << "," << target_Y << "," << target_Z << ")"
                                << " local=(" << local_X << "," << local_Y << "," << local_Z << ")"
                                << " q=(" << q0 << "," << q1 << "," << q2 << ")"
                                << " motor=(" << m0 << "," << m1 << "," << m2 << ")\n";
                    }
                }
            }
        }
    }
    delete robot;
    return 0;
}
