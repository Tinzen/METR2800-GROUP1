// Software-in-the-loop harness: runs the real firmware against a simple
// plant model of the outtake slide + timing model of a 16 MHz Nano.
//
// Usage: ./sim [seconds] [script.txt]
//   script.txt lines: "<ms> <serial command>"  (sent to the firmware at that time)
#include "Arduino.h"
#include "Wire.h"
#include <vector>
#include <string>
#include <deque>
#include <random>

SerialMock Serial;
TwoWire Wire;

// ---------------- time model ----------------
static double simUs = 0.0;
static std::mt19937 rng(42);
static void advanceUs(double us);

static bool inSetup = false;   // busy-wait loops in setup() (bench handshake) need time to pass
unsigned long millis() { if (inSetup) advanceUs(100.0); return (unsigned long)(simUs / 1000.0); }
unsigned long micros() { return (unsigned long)simUs; }
void delay(unsigned long ms) { advanceUs(ms * 1000.0); }

// ---------------- serial model (115200 baud, 64 B TX buffer) ----------------
static double txBufBytes = 0.0;           // bytes waiting in TX buffer
static const double BYTE_US = 1e6 / 11520.0;
static std::deque<char> rxQueue;
static std::string outLine;

size_t SerialMock::write(const char* s) {
  size_t n = strlen(s);
  for (size_t i = 0; i < n; i++) {
    if (txBufBytes >= 63.0) {             // buffer full -> blocks until a byte drains
      advanceUs(BYTE_US);
    }
    txBufBytes += 1.0;
    advanceUs(4.0);                        // per-char CPU cost
    fputc(s[i], stdout);
  }
  return n;
}
int SerialMock::available() { return (int)rxQueue.size(); }
int SerialMock::read() {
  if (rxQueue.empty()) return -1;
  char c = rxQueue.front(); rxQueue.pop_front(); return c;
}

// ---------------- pins ----------------
static int pinLevel[32];
static int pwmValue[32];
void pinMode(int, int) {}
void digitalWrite(int p, int v) { if (p >= 0 && p < 32) pinLevel[p] = v; advanceUs(5); }
int digitalRead(int p) { return pinLevel[p]; }
void analogWrite(int p, int v) { if (p >= 0 && p < 32) pwmValue[p] = v; advanceUs(5); }

// ---------------- servo driver ----------------
static uint16_t servoCounts[16];
void sim_pwm_write(uint8_t ch, uint16_t off) {
  if (ch < 16) servoCounts[ch] = off;
  advanceUs(160.0);                        // ~7 bytes at 400 kHz I2C
}

// ---------------- outtake plant ----------------
// ticks + ticks/s. Loaded free speed ~2600 t/s at full PWM.
struct Slide {
  double x = 0.0, v = 0.0;
  double xMax = 6000.0;          // physical end of travel
  double contact = 4300.0;       // bucket touches the hopper pole here
} plant;
static long encOffset = 0;
// plant knobs (override with env vars SIM_GRAVITY / SIM_FRICTION / SIM_XMAX)
static double simGravity = 0.07, simFriction = 0.045;

static double drive() {
  // pins 8/9 direction, 11 PWM (RobotConfig.h)
  double u = pwmValue[11] / 255.0;
  if (pinLevel[8] && !pinLevel[9]) return u;
  if (!pinLevel[8] && pinLevel[9]) return -u;
  return 0.0;
}

static void plantStep(double dt) {
  const double vFree = 2600.0, tau = 0.06;
  const double k = vFree / tau;                   // t/s^2 per unit drive
  double F = (vFree * drive() - plant.v) / tau;   // motor
  double Fg = -simGravity * k;                    // gravity pulls it back down
  double Ff = simFriction * k;                    // coulomb friction
  if (plant.x > plant.contact) Fg -= 0.05 * k * std::min(1.0, (plant.x - plant.contact) / 200.0); // tipping bucket
  double a;
  if (fabs(plant.v) < 5.0) {
    double net = F + Fg;
    if (fabs(net) <= Ff) { plant.v = 0.0; a = 0.0; }
    else a = net - (net > 0 ? Ff : -Ff);
  } else {
    a = F + Fg - (plant.v > 0 ? Ff : -Ff);
  }
  plant.v += a * dt;
  plant.x += plant.v * dt;
  if (plant.x < 0.0) { plant.x = 0.0; if (plant.v < 0) plant.v = 0; }
  if (plant.x > plant.xMax) { plant.x = plant.xMax; if (plant.v > 0) plant.v = 0; }
}

long sim_encoder_read() { return -(long)floor(plant.x) + encOffset; }
void sim_encoder_write(long v) { encOffset = v + (long)floor(plant.x); }

static void advanceUs(double us) {
  // drain serial
  txBufBytes -= us / BYTE_US;
  if (txBufBytes < 0) txBufBytes = 0;
  // integrate plant in 100 us sub-steps
  double t = us;
  while (t > 0) {
    double h = t > 100.0 ? 100.0 : t;
    plantStep(h * 1e-6);
    t -= h;
  }
  simUs += us;
}

// ---------------- firmware ----------------
void setup();
void loop();

int main(int argc, char** argv) {
  double seconds = argc > 1 ? atof(argv[1]) : 25.0;
  std::vector<std::pair<unsigned long, std::string>> script;
  if (argc > 2) {
    FILE* f = fopen(argv[2], "r");
    char line[128];
    while (f && fgets(line, sizeof line, f)) {
      unsigned long ms; char cmd[100];
      if (sscanf(line, "%lu %99[^\n]", &ms, cmd) == 2) script.push_back({ms, cmd});
    }
    if (f) fclose(f);
  }
  size_t si = 0;
  std::uniform_real_distribution<double> jitter(0.0, 80.0);

  if (getenv("SIM_X0")) plant.x = atof(getenv("SIM_X0"));
  if (getenv("SIM_GRAVITY")) simGravity = atof(getenv("SIM_GRAVITY"));
  if (getenv("SIM_FRICTION")) simFriction = atof(getenv("SIM_FRICTION"));
  if (getenv("SIM_XMAX")) plant.xMax = atof(getenv("SIM_XMAX"));
  if (getenv("SIM_CONTACT")) plant.contact = atof(getenv("SIM_CONTACT"));
  inSetup = true; setup(); inSetup = false;
  while (simUs < seconds * 1e6) {
    while (si < script.size() && millis() >= script[si].first) {
      for (char c : script[si].second) rxQueue.push_back(c);
      rxQueue.push_back('\n');
      si++;
    }
    loop();
    advanceUs(320.0 + jitter(rng));       // AVR float math etc.
  }
  fprintf(stderr, "sim end: slide x=%.0f v=%.0f servo counts R=%u L=%u S=%u\n", plant.x, plant.v, servoCounts[0], servoCounts[1], servoCounts[2]);
  return 0;
}
