/*
 * teensy_cartpole_sim.ino
 *
 * Physics-only cart-pole validation sketch for Teensy 4.1.  Runs a
 * semi-implicit Euler integrator for the cart-pole equations of motion
 * and streams the trajectory to a Mac over UDP.  No motor, no Modbus,
 * no real-time constraint — the simulation runs as fast as possible.
 *
 * Protocol and networking style match teensy_orca_bridge.ino.
 * See CARTPOLE_VALIDATION_README.md for the full specification.
 */

#include <QNEthernet.h>
#include <math.h>
using namespace qindesign::network;

// =====================================================================
// Network configuration (same as teensy_orca_bridge)
// =====================================================================
IPAddress teensyIP   (192, 168, 1, 177);
IPAddress subnetMask (255, 255, 255, 0);
IPAddress gateway    (192, 168, 1, 1);

constexpr unsigned int LOCAL_PORT = 8888;

EthernetUDP Udp;
IPAddress macIP;
uint16_t  macPort = 0;

// =====================================================================
// Simulation parameters (set by SIM_CONFIG)
// =====================================================================
double sim_m_c = 1.0;
double sim_m_p = 0.1;
double sim_l   = 0.5;
double sim_g   = 9.81;
bool   sim_configured = false;

// =====================================================================
// Simulation state (set by SIM_INIT, updated during SIM_RUN)
// =====================================================================
double sim_x, sim_theta, sim_xdot, sim_thetadot;
bool   sim_initialized = false;

// =====================================================================
// Run parameters (set by SIM_RUN)
// =====================================================================
double   sim_dt;
uint64_t sim_total_steps;
int      sim_sample_stride;
int      sim_fx_mode;
double   sim_fx_p1, sim_fx_p2;

// =====================================================================
// Sample output buffer — batch up to 8 samples per UDP packet to avoid
// fragmentation and improve throughput.
// =====================================================================
constexpr int MAX_SAMPLES_PER_PACKET = 8;
char pkt_buf[1500];
int  pkt_len     = 0;
int  pkt_samples = 0;

void flush_packet() {
  if (pkt_len > 0 && macPort != 0) {
    Udp.beginPacket(macIP, macPort);
    Udp.write((const uint8_t*)pkt_buf, pkt_len);
    Udp.endPacket();
  }
  pkt_len     = 0;
  pkt_samples = 0;
}

void append_sample(uint64_t step_idx, double t,
                   double sx, double stheta,
                   double sxdot, double sthetadot, double sfx) {
  // Wrap theta to (-pi, pi] for output only — integrator uses unwrapped.
  double th = fmod(stheta, 2.0 * M_PI);
  if (th >  M_PI)  th -= 2.0 * M_PI;
  if (th <= -M_PI) th += 2.0 * M_PI;

  int remaining = (int)sizeof(pkt_buf) - pkt_len;
  int n = snprintf(pkt_buf + pkt_len, remaining,
                   "SIM_SAMPLE %llu %.10g %.10g %.10g %.10g %.10g %.10g\n",
                   (unsigned long long)step_idx, t,
                   sx, th, sxdot, sthetadot, sfx);
  if (n > 0 && n < remaining) {
    pkt_len += n;
    pkt_samples++;
  }
  if (pkt_samples >= MAX_SAMPLES_PER_PACKET) {
    flush_packet();
  }
}

// =====================================================================
// UDP helpers (style matches teensy_orca_bridge)
// =====================================================================
void send_reply(const char* msg) {
  if (macPort == 0) return;
  Udp.beginPacket(macIP, macPort);
  Udp.write(msg);
  Udp.endPacket();
}

void send_ack(const char* cmd) {
  char buf[64];
  snprintf(buf, sizeof(buf), "ACK %s", cmd);
  send_reply(buf);
}

void send_error(const char* msg) {
  char buf[160];
  snprintf(buf, sizeof(buf), "ERROR %s", msg);
  send_reply(buf);
}

// =====================================================================
// Force function f_x(t)
// =====================================================================
double compute_fx(double t) {
  switch (sim_fx_mode) {
    case 1:  return sim_fx_p1;
    case 2:  return sim_fx_p1 * sin(2.0 * M_PI * sim_fx_p2 * t);
    case 3:  return (t < sim_fx_p2) ? 0.0 : sim_fx_p1;
    case 4:  return (t < sim_fx_p2) ? sim_fx_p1 : 0.0;
    default: return 0.0;  // mode 0 and anything unrecognised
  }
}
// =====================================================================
// Cart-pole continuous dynamics: state s = (x, theta, xdot, thetadot)
// =====================================================================
static inline void dynamics(const double s[4], double fx, double ds[4]) {
  double theta    = s[1];
  double xdot     = s[2];
  double thetadot = s[3];

  double sn = sin(theta);
  double cs = cos(theta);
  double D  = sim_m_c + sim_m_p * sn * sn;

  double xdd = (fx + sim_m_p * sn
                * (sim_l * thetadot * thetadot + sim_g * cs)) / D;
  double tdd = (-fx * cs
                - sim_m_p * sim_l * thetadot * thetadot * cs * sn
                - (sim_m_c + sim_m_p) * sim_g * sn) / (sim_l * D);

  ds[0] = xdot;
  ds[1] = thetadot;
  ds[2] = xdd;
  ds[3] = tdd;
}

// =====================================================================
// Simulation core — classical RK4
// =====================================================================
void run_simulation() {
  uint32_t wall_start = micros();

  char msg[128];
  snprintf(msg, sizeof(msg), "SIM_START %llu %lu",
           (unsigned long long)sim_total_steps, (unsigned long)wall_start);
  send_reply(msg);

  pkt_len     = 0;
  pkt_samples = 0;
  elapsedMicros since_yield;

  for (uint64_t k = 0; k < sim_total_steps; k++) {
    double t_k    = (double)k * sim_dt;
    double t_mid  = t_k + 0.5 * sim_dt;
    double t_end  = t_k + sim_dt;

    double fx_k   = compute_fx(t_k);
    double fx_mid = compute_fx(t_mid);
    double fx_end = compute_fx(t_end);

    double y[4]  = { sim_x, sim_theta, sim_xdot, sim_thetadot };
    double k1[4], k2[4], k3[4], k4[4], ytmp[4];

    dynamics(y, fx_k, k1);
    for (int i = 0; i < 4; i++) ytmp[i] = y[i] + 0.5 * sim_dt * k1[i];
    dynamics(ytmp, fx_mid, k2);
    for (int i = 0; i < 4; i++) ytmp[i] = y[i] + 0.5 * sim_dt * k2[i];
    dynamics(ytmp, fx_mid, k3);
    for (int i = 0; i < 4; i++) ytmp[i] = y[i] + sim_dt * k3[i];
    dynamics(ytmp, fx_end, k4);

    double sixth = sim_dt / 6.0;
    sim_x        += sixth * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
    sim_theta    += sixth * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
    sim_xdot     += sixth * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]);
    sim_thetadot += sixth * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]);

    if (!isfinite(sim_x) || !isfinite(sim_theta) ||
        !isfinite(sim_xdot) || !isfinite(sim_thetadot)) {
      flush_packet();
      send_error("NAN_STATE");
      return;
    }

    if (k % sim_sample_stride == 0) {
      double t_out = (double)(k + 1) * sim_dt;
      append_sample(k, t_out, sim_x, sim_theta,
                    sim_xdot, sim_thetadot, fx_k);
    }

    if (since_yield >= 1000) {
      since_yield = 0;
      int sz = Udp.parsePacket();
      if (sz > 0) {
        char abuf[64];
        int n = Udp.read(abuf, sizeof(abuf) - 1);
        if (n > 0) {
          abuf[n] = 0;
          if (strncmp(abuf, "ABORT", 5) == 0) {
            flush_packet();
            send_error("ABORTED");
            return;
          }
        }
      }
    }
  }

  flush_packet();

  uint32_t wall_elapsed = micros() - wall_start;
  snprintf(msg, sizeof(msg), "SIM_DONE %llu %lu",
           (unsigned long long)sim_total_steps, (unsigned long)wall_elapsed);
  send_reply(msg);
}

// =====================================================================
// Command handler
// =====================================================================
void handle_command(char* line) {
  macIP   = Udp.remoteIP();
  macPort = Udp.remotePort();

  char* tok = strtok(line, " \r\n");
  if (!tok) return;

  if (strcmp(tok, "PING") == 0) {
    send_ack("PING");
  }
  else if (strcmp(tok, "SIM_CONFIG") == 0) {
    char* a = strtok(NULL, " \r\n");
    char* b = strtok(NULL, " \r\n");
    char* c = strtok(NULL, " \r\n");
    char* d = strtok(NULL, " \r\n");
    if (a && b && c && d) {
      sim_m_c = strtod(a, NULL);
      sim_m_p = strtod(b, NULL);
      sim_l   = strtod(c, NULL);
      sim_g   = strtod(d, NULL);
      sim_configured = true;
      send_ack("SIM_CONFIG");
    } else {
      send_error("SIM_CONFIG_BAD_ARGS");
    }
  }
  else if (strcmp(tok, "SIM_INIT") == 0) {
    char* a = strtok(NULL, " \r\n");
    char* b = strtok(NULL, " \r\n");
    char* c = strtok(NULL, " \r\n");
    char* d = strtok(NULL, " \r\n");
    if (a && b && c && d) {
      sim_x        = strtod(a, NULL);
      sim_theta    = strtod(b, NULL);
      sim_xdot     = strtod(c, NULL);
      sim_thetadot = strtod(d, NULL);
      sim_initialized = true;
      send_ack("SIM_INIT");
    } else {
      send_error("SIM_INIT_BAD_ARGS");
    }
  }
  else if (strcmp(tok, "SIM_RUN") == 0) {
    if (!sim_configured)  { send_error("NOT_CONFIGURED");  return; }
    if (!sim_initialized) { send_error("NOT_INITIALIZED"); return; }

    char* a = strtok(NULL, " \r\n");  // duration_s
    char* b = strtok(NULL, " \r\n");  // dt_s
    char* c = strtok(NULL, " \r\n");  // fx_mode
    char* d = strtok(NULL, " \r\n");  // fx_param1
    char* e = strtok(NULL, " \r\n");  // fx_param2
    char* f = strtok(NULL, " \r\n");  // sample_stride
    if (a && b && c && d && e && f) {
      double duration   = strtod(a, NULL);
      sim_dt            = strtod(b, NULL);
      sim_fx_mode       = atoi(c);
      sim_fx_p1         = strtod(d, NULL);
      sim_fx_p2         = strtod(e, NULL);
      sim_sample_stride = atoi(f);
      if (sim_sample_stride < 1) sim_sample_stride = 1;
      sim_total_steps   = (uint64_t)(duration / sim_dt + 0.5);
      run_simulation();
    } else {
      send_error("SIM_RUN_BAD_ARGS");
    }
  }
  else if (strcmp(tok, "ABORT") == 0) {
    // Handled inside run_simulation's yield check; if we reach here
    // no simulation is running, so there is nothing to abort.
  }
  else {
    send_error("UNKNOWN_COMMAND");
  }
}

// =====================================================================
// Arduino entry points
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("teensy_cartpole_sim starting");

  if (!Ethernet.begin(teensyIP, subnetMask, gateway)) {
    Serial.println("ERROR: Ethernet.begin() failed.");
    while (true) delay(1000);
  }
  Udp.begin(LOCAL_PORT);

  Serial.print("Listening on UDP ");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.println(LOCAL_PORT);
}

void loop() {
  int sz = Udp.parsePacket();
  if (sz > 0) {
    char buf[256];
    int n = Udp.read(buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = 0;
      handle_command(buf);
    }
  }
}
