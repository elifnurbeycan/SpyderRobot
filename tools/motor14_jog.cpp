/*
 * motor14_jog.cpp — Motor 14'u klavyeyle ileri/geri sur, enc degerlerini kaydet
 *
 * Firmware: mega_full_bridge.cpp (M <id> <enc> ve R <id> komutlari)
 *
 * Derleme:
 *   g++ -std=c++17 -O2 -o motor14_jog motor14_jog.cpp
 * Calistirma:
 *   ./motor14_jog /dev/ttyUSB0
 *
 * Klavye:
 *   W  -> ileri (hedef enc artar)
 *   A  -> geri  (hedef enc azalir)
 *   S  -> hedefi bulundugu present enc'e sabitle (dur)
 *   Q  -> cikis
 *
 * Her dongude present enc hem ekrana basilir hem motor14_log.csv'ye yazilir.
 */

#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>

// --- Ayarlar ---
const int    MOTOR_ID   = 14;
const int    ENC_MIN    = 3362;   // femur_limits.py — ID14 mekanik alt sinir
const int    ENC_MAX    = 4091;   // femur_limits.py — ID14 mekanik ust sinir
const int    ENC_START  = 3812;   // stance (home_position.py)
const int    STEP       = 20;     // her tus basiminda hedef degisimi (enc adim)
const int    LOOP_MS    = 100;    // 10 Hz (half-duplex okuma icin rahat)

int g_fd = -1;

int openSerial(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { perror("open"); return -1; }
    struct termios t{};
    tcgetattr(fd, &t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    cfmakeraw(&t);
    t.c_cc[VMIN] = 0;
    t.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &t);
    return fd;
}

void serialWrite(const std::string &s) {
    if (write(g_fd, s.c_str(), s.size()) < 0) perror("write");
}

// Tek satir yanit oku (\n'e kadar), timeout ms
std::string readLine(int timeout_ms) {
    std::string line;
    struct timespec t0, tn;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    char c;
    while (true) {
        int n = read(g_fd, &c, 1);
        if (n == 1) {
            if (c == '\n') break;
            if (c != '\r') line += c;
        } else {
            clock_gettime(CLOCK_MONOTONIC, &tn);
            double ms = (tn.tv_sec - t0.tv_sec) * 1000.0
                      + (tn.tv_nsec - t0.tv_nsec) / 1e6;
            if (ms > timeout_ms) break;
            usleep(500);
        }
    }
    return line;
}

// R <id> gonder, "P <id> <pos>" yanitindan pos'u dondur (-1 = hata)
int readEnc(int id) {
    tcflush(g_fd, TCIFLUSH);
    serialWrite("R " + std::to_string(id) + "\n");
    // Birkac satir gelebilir (bosluk/artik); "P <id> <pos>" olani yakala
    for (int tries = 0; tries < 4; tries++) {
        std::string line = readLine(120);
        int rid, pos;
        if (sscanf(line.c_str(), "P %d %d", &rid, &pos) == 2 && rid == id)
            return pos;
        if (line.empty()) break;
    }
    return -1;
}

void moveTo(int id, int enc) {
    serialWrite("M " + std::to_string(id) + " " + std::to_string(enc) + "\n");
    tcflush(g_fd, TCIFLUSH);   // OK yanitini yut
}

struct Keyboard {
    struct termios orig{};
    void init() {
        tcgetattr(STDIN_FILENO, &orig);
        struct termios raw = orig;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    void restore() { tcsetattr(STDIN_FILENO, TCSANOW, &orig); }
    int read() {
        char buf[4] = {0};
        int n = ::read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0) return 0;
        char c = toupper(buf[0]);
        return c;
    }
};

double now_sec() {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv) {
    const char *dev = (argc > 1) ? argv[1] : "/dev/ttyUSB0";
    g_fd = openSerial(dev);
    if (g_fd < 0) return 1;
    printf("Seri port acik: %s\n", dev);
    sleep(2);                      // Arduino reset
    tcflush(g_fd, TCIOFLUSH);

    FILE *log = fopen("motor14_log.csv", "w");
    fprintf(log, "t_sec,hedef_enc,present_enc,tus\n");
    fflush(log);

    printf("Motor %d jog. W=ileri  A=geri  S=dur  Q=cikis\n", MOTOR_ID);
    printf("Limitler: %d..%d, baslangic %d\n\n", ENC_MIN, ENC_MAX, ENC_START);

    Keyboard kb; kb.init();

    int hedef = ENC_START;
    moveTo(MOTOR_ID, hedef);
    double t0 = now_sec();
    double loop_start = t0;

    while (true) {
        int key = kb.read();
        char tus = ' ';

        if (key == 'Q') break;
        if (key == 'W') { hedef += STEP; tus = 'W'; }
        if (key == 'A') { hedef -= STEP; tus = 'A'; }
        if (key == 'S') {                      // present'e sabitle
            int p = readEnc(MOTOR_ID);
            if (p >= 0) hedef = p;
            tus = 'S';
        }
        hedef = std::clamp(hedef, ENC_MIN, ENC_MAX);

        moveTo(MOTOR_ID, hedef);
        usleep(3000);                 // M'in echo/OK'i otursun
        tcflush(g_fd, TCIOFLUSH);     // hatti tamamen temizle
        int present = readEnc(MOTOR_ID);

        double t = now_sec() - t0;
        printf("\rt=%6.2f  hedef=%4d  present=%4d  tus=%c   ",
               t, hedef, present, tus);
        fflush(stdout);

        fprintf(log, "%.3f,%d,%d,%c\n", t, hedef, present, tus);
        fflush(log);

        double elapsed = (now_sec() - loop_start) * 1000.0;
        int remain = LOOP_MS - (int)elapsed;
        if (remain > 0) usleep(remain * 1000);
        loop_start = now_sec();
    }

    kb.restore();
    fclose(log);
    close(g_fd);
    printf("\nCikis. Kayit: motor14_log.csv\n");
    return 0;
}
