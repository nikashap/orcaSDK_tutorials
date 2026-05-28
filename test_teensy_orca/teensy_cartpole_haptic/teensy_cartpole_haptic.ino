/*
 * teensy_cartpole_haptic.ino
 *
 * Step 4: Integrated haptic cart-pole loop on Teensy 4.1.
 *
 * Merges the validated pendulum physics (Step 1) with the Orca Modbus
 * bridge (Step 2).  A human drives the motor shaft ("cart"); the Teensy
 * reads shaft kinematics, integrates the virtual pendulum via RK4,
 * and commands the motor to render the pendulum reaction force.
 *
 * Mac sends binary UDP commands (see HAPTIC_LOOP_README.md).
 * Teensy streams packed binary telemetry back.
 *
 * Hardware: Teensy 4.1 + Ethernet kit (QNEthernet),
 *           MAX490 RS-422 on Serial1 (pins 0 RX, 1 TX),
 *           IrisDynamics Orca motor on Modbus RTU.
 */

#include <QNEthernet.h>
#include <math.h>
using namespace qindesign::network;

// =====================================================================
// Network
// =====================================================================
IPAddress teensyIP   (192, 168, 1, 177);
IPAddress subnetMask (255, 255, 255, 0);
IPAddress gateway    (192, 168, 1, 1);

constexpr uint16_t LOCAL_PORT = 8888;

EthernetUDP Udp;
IPAddress macIP;
uint16_t  macPort = 0;

// =====================================================================
// Modbus configuration (from teensy_orca_bridge)
// =====================================================================
constexpr uint8_t  MOTOR_ADDR            = 0x01;
constexpr uint32_t DEFAULT_BAUD          = 1000000;
constexpr uint32_t DEFAULT_INTERFRAME_US = 100;

constexpr uint8_t FC_READ_HOLDING        = 0x03;
constexpr uint8_t FC_WRITE_SINGLE        = 0x06;
constexpr uint8_t FC_WRITE_MULTIPLE      = 0x10;
constexpr uint8_t FC_MANAGE_STREAM       = 0x41;
constexpr uint8_t FC_MOTOR_CMD_STREAM    = 0x64;

constexpr uint8_t SUB_SLEEP              = 0x00;
constexpr uint8_t SUB_FORCE_CTRL         = 0x1C;

constexpr uint16_t REG_SHAFT_SPEED       = 344;
constexpr uint16_t REG_SHAFT_ACCEL       = 346;

// Autozero registers (from Orca SDK / Reference Manual)
// Verify these against RM220115 Table 2 if autozero misbehaves.
constexpr uint16_t REG_ZERO_MODE            = 171;
constexpr uint16_t REG_AUTO_ZERO_FORCE_N    = 172;
constexpr uint16_t REG_AUTO_ZERO_SPEED_MMPS = 177;
constexpr uint16_t REG_AUTO_ZERO_EXIT_MODE  = 173;
constexpr uint16_t REG_CTRL_REG_3           = 3;    // write desired mode here
constexpr uint16_t REG_MODE_OF_OPERATION    = 317;  // read-only: confirms current mode
constexpr uint16_t REG_ERROR_0              = 432;

constexpr uint16_t ZERO_MODE_AUTO_ZERO      = 2;
constexpr uint16_t MOTOR_MODE_SLEEP         = 1;
constexpr uint16_t MOTOR_MODE_AUTO_ZERO     = 55;

// =====================================================================
// Binary protocol — opcodes (Mac → Teensy)
// =====================================================================
constexpr uint8_t CMD_PING     = 0x01;
constexpr uint8_t CMD_AUTOZERO = 0x02;
constexpr uint8_t CMD_CONFIG   = 0x03;
constexpr uint8_t CMD_INIT     = 0x04;
constexpr uint8_t CMD_BEGIN    = 0x05;
constexpr uint8_t CMD_END      = 0x06;
constexpr uint8_t CMD_SLEEP    = 0x07;

// Reply opcodes (Teensy → Mac)
constexpr uint8_t REPLY_ACK   = 0xA1;
constexpr uint8_t REPLY_ERROR = 0xA2;
constexpr uint8_t TELE_BATCH  = 0xB0;
constexpr uint8_t TELE_FINAL  = 0xB1;

// Error codes
constexpr uint8_t ERR_BADOP           = 0x01;
constexpr uint8_t ERR_BADLEN          = 0x02;
constexpr uint8_t ERR_MOTOR_FAULT     = 0x03;
constexpr uint8_t ERR_SIGN_CHECK      = 0x04;
constexpr uint8_t ERR_NAN_STATE       = 0x05;
constexpr uint8_t ERR_MODBUS_TIMEOUT  = 0x06;
constexpr uint8_t ERR_NOT_CONFIGURED  = 0x07;

// Expected payload sizes per opcode
constexpr size_t PAYLOAD_PING     = 0;
constexpr size_t PAYLOAD_AUTOZERO = 0;
constexpr size_t PAYLOAD_CONFIG   = 28;  // 7 × float
constexpr size_t PAYLOAD_INIT     = 16;  // 4 × float
constexpr size_t PAYLOAD_BEGIN    = 0;
constexpr size_t PAYLOAD_END      = 0;
constexpr size_t PAYLOAD_SLEEP_CMD= 0;

// =====================================================================
// Physics parameters (set by CMD_CONFIG)
// =====================================================================
double par_m_c          = 0.0;
double par_m_p          = 0.0;
double par_m_s          = 0.0;
double par_l            = 0.0;
double par_g            = 9.81;
double par_max_force_mN = 0.0;
uint32_t par_loop_us    = 1800;
bool configured = false;

// =====================================================================
// Physics state
// =====================================================================
double st_x        = 0.0;
double st_xdot     = 0.0;
double st_xddot    = 0.0;
double st_theta    = 0.0;
double st_thetadot = 0.0;
bool   state_initialized = false;

// =====================================================================
// Telemetry sample (packed binary, little-endian)
//
// Python struct format: '<II9fI'
// Total: 4+4 + 9*4 + 4 = 48 bytes
// =====================================================================
struct __attribute__((packed)) Sample {
  uint32_t cycle;
  uint32_t t_meas_us;
  float    x, xdot, xddot;
  float    theta, thetadot;
  float    fx;
  float    fpc;
  float    f_command_mN;
  float    force_sensed_mN;  // motor_tele.force_mn (lags ~3 frames)
  uint32_t loop_us;
};
static_assert(sizeof(Sample) == 48, "Sample struct packing");

constexpr int MAX_SAMPLES_PER_PACKET = 8;
Sample   tele_buf[MAX_SAMPLES_PER_PACKET];
uint8_t  tele_count = 0;

// =====================================================================
// Motor telemetry from 0x64 response
// =====================================================================
struct MotorTelemetry {
  int32_t  position_um;
  int32_t  force_mn;
  uint16_t power_w;
  int8_t   temperature_c;
  uint16_t voltage_mv;
  uint16_t errors;
  bool     valid;
};
MotorTelemetry motor_tele = {};

// =====================================================================
// Result of one extended-mode motor exchange (defined here, before the
// first function, so the Arduino auto-generated prototype for
// motor_exchange() can see the type).
// =====================================================================
struct MotorExchange {
  int32_t  position_um;
  int32_t  speed_mmps;
  int32_t  accel_mmpss;
  uint16_t errors;
  uint32_t t_meas_us;
  bool     ok;
};

// =====================================================================
// State machine
// =====================================================================
enum class Phase : uint8_t {
  IDLE,
  CONNECTED,
  CONFIGURED,
  RUNNING
};
Phase phase = Phase::IDLE;

uint32_t loop_cycle      = 0;
int32_t  prev_force_mN   = 0;   // force command from previous cycle
bool     motor_connected  = false;
uint32_t current_interframe_us = DEFAULT_INTERFRAME_US;

// =====================================================================
// CRC-16 Modbus (from teensy_orca_bridge)
// =====================================================================
uint16_t modbus_crc(const uint8_t* buf, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t pos = 0; pos < len; ++pos) {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; --i) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else              { crc >>= 1; }
    }
  }
  return crc;
}

size_t append_crc(uint8_t* buf, size_t len) {
  uint16_t crc = modbus_crc(buf, len);
  buf[len]     = crc & 0xFF;
  buf[len + 1] = (crc >> 8) & 0xFF;
  return len + 2;
}

bool check_crc(const uint8_t* buf, size_t len) {
  if (len < 4) return false;
  uint16_t calc = modbus_crc(buf, len - 2);
  uint16_t recv = buf[len - 2] | (buf[len - 1] << 8);
  return calc == recv;
}

// =====================================================================
// Serial / Modbus I/O (from teensy_orca_bridge)
// =====================================================================
void interframe_delay() {
  delayMicroseconds(current_interframe_us);
}

int modbus_transact(const uint8_t* tx, size_t tx_len,
                    uint8_t* rx, size_t rx_max,
                    uint32_t timeout_us,
                    size_t expected_len = 0) {
  interframe_delay();
  while (Serial1.available()) Serial1.read();

  Serial1.write(tx, tx_len);
  Serial1.flush();

  elapsedMicros t;
  size_t got = 0;
  size_t target = (expected_len > 0 && expected_len <= rx_max)
                  ? expected_len : rx_max;
  while (t < timeout_us && got < target) {
    if (Serial1.available()) {
      rx[got++] = Serial1.read();
      t = 0;
      if (got >= target) break;
    }
  }
  return got;
}

// =====================================================================
// Modbus frame builders (from teensy_orca_bridge)
// =====================================================================
size_t build_write_single(uint8_t* buf, uint16_t addr, uint16_t value) {
  buf[0] = MOTOR_ADDR;
  buf[1] = FC_WRITE_SINGLE;
  buf[2] = (addr >> 8) & 0xFF;
  buf[3] = addr & 0xFF;
  buf[4] = (value >> 8) & 0xFF;
  buf[5] = value & 0xFF;
  return append_crc(buf, 6);
}

size_t build_read_holding(uint8_t* buf, uint16_t addr, uint16_t count) {
  buf[0] = MOTOR_ADDR;
  buf[1] = FC_READ_HOLDING;
  buf[2] = (addr >> 8) & 0xFF;
  buf[3] = addr & 0xFF;
  buf[4] = (count >> 8) & 0xFF;
  buf[5] = count & 0xFF;
  return append_crc(buf, 6);
}

bool parse_read_holding_2reg_int32(const uint8_t* buf, size_t len,
                                   int32_t* out) {
  if (len != 9)                  return false;
  if (buf[0] != MOTOR_ADDR)      return false;
  if (buf[1] != FC_READ_HOLDING) return false;
  if (buf[2] != 4)               return false;
  if (!check_crc(buf, len))      return false;
  uint16_t lo = ((uint16_t)buf[3] << 8) | buf[4];
  uint16_t hi = ((uint16_t)buf[5] << 8) | buf[6];
  uint32_t v  = ((uint32_t)hi << 16) | lo;
  *out = (int32_t)v;
  return true;
}

size_t build_manage_stream(uint8_t* buf, bool enable,
                           uint32_t baud, uint16_t delay_us) {
  buf[0] = MOTOR_ADDR;
  buf[1] = FC_MANAGE_STREAM;
  buf[2] = enable ? 0xFF : 0x00;
  buf[3] = 0x00;
  buf[4] = (baud >> 24) & 0xFF;
  buf[5] = (baud >> 16) & 0xFF;
  buf[6] = (baud >> 8)  & 0xFF;
  buf[7] = baud & 0xFF;
  buf[8] = (delay_us >> 8) & 0xFF;
  buf[9] = delay_us & 0xFF;
  return append_crc(buf, 10);
}

size_t build_motor_stream(uint8_t* buf, uint8_t sub_code, int32_t data) {
  buf[0] = MOTOR_ADDR;
  buf[1] = FC_MOTOR_CMD_STREAM;
  buf[2] = sub_code;
  buf[3] = (data >> 24) & 0xFF;
  buf[4] = (data >> 16) & 0xFF;
  buf[5] = (data >> 8)  & 0xFF;
  buf[6] = data & 0xFF;
  return append_crc(buf, 7);
}

bool parse_motor_stream_response(const uint8_t* buf, size_t len) {
  if (len != 19) return false;
  if (buf[0] != MOTOR_ADDR) return false;
  if (buf[1] != FC_MOTOR_CMD_STREAM) return false;
  if (!check_crc(buf, len)) return false;

  motor_tele.position_um   = ((int32_t)buf[2]  << 24) | ((int32_t)buf[3]  << 16)
                            | ((int32_t)buf[4]  << 8)  |  (int32_t)buf[5];
  motor_tele.force_mn      = ((int32_t)buf[6]  << 24) | ((int32_t)buf[7]  << 16)
                            | ((int32_t)buf[8]  << 8)  |  (int32_t)buf[9];
  motor_tele.power_w       = ((uint16_t)buf[10] << 8) | buf[11];
  motor_tele.temperature_c = (int8_t)buf[12];
  motor_tele.voltage_mv    = ((uint16_t)buf[13] << 8) | buf[14];
  motor_tele.errors        = ((uint16_t)buf[15] << 8) | buf[16];
  motor_tele.valid         = true;
  return true;
}

// =====================================================================
// Motor verification (from teensy_orca_bridge)
// =====================================================================
bool verify_motor_present() {
  uint8_t tx[8];
  tx[0] = MOTOR_ADDR;
  tx[1] = FC_READ_HOLDING;
  tx[2] = 0x01; tx[3] = 0x96;  // register 406 (serial number)
  tx[4] = 0x00; tx[5] = 0x02;
  size_t tx_len = append_crc(tx, 6);

  uint8_t rx[32];
  int n = modbus_transact(tx, tx_len, rx, sizeof(rx), 50000, 9);
  if (n != 9)                    return false;
  if (rx[0] != MOTOR_ADDR)       return false;
  if (rx[1] != FC_READ_HOLDING)  return false;
  if (rx[2] != 4)                return false;
  return check_crc(rx, n);
}

// =====================================================================
// UDP reply helpers (binary)
// =====================================================================
void send_ack(uint8_t cmd_opcode) {
  if (macPort == 0) return;
  uint8_t buf[2] = { REPLY_ACK, cmd_opcode };
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, 2);
  Udp.endPacket();
}

// PING reply includes Teensy's micros() for clock synchronization.
// Format: 0xA1 0x01 <uint32_t micros LE>  (6 bytes)
void send_ping_ack() {
  if (macPort == 0) return;
  uint32_t t = micros();
  uint8_t buf[6];
  buf[0] = REPLY_ACK;
  buf[1] = CMD_PING;
  memcpy(buf + 2, &t, 4);  // little-endian on ARM
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, 6);
  Udp.endPacket();
}

void send_error(uint8_t err_code) {
  if (macPort == 0) return;
  uint8_t buf[2] = { REPLY_ERROR, err_code };
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, 2);
  Udp.endPacket();
}

void send_error_with_fault(uint8_t err_code, uint16_t fault_bits) {
  if (macPort == 0) return;
  uint8_t buf[4] = {
    REPLY_ERROR, err_code,
    (uint8_t)(fault_bits & 0xFF),
    (uint8_t)((fault_bits >> 8) & 0xFF)
  };
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, 4);
  Udp.endPacket();
}

// Debug text message (prefixed 0xD0 so Python can distinguish)
void send_info(const char* msg) {
  if (macPort == 0) return;
  uint8_t hdr = 0xD0;
  Udp.beginPacket(macIP, macPort);
  Udp.write(&hdr, 1);
  Udp.write(msg);
  Udp.endPacket();
}

// =====================================================================
// Telemetry batching
// =====================================================================
void flush_telemetry() {
  if (tele_count == 0 || macPort == 0) return;
  uint8_t hdr[2] = { TELE_BATCH, tele_count };
  Udp.beginPacket(macIP, macPort);
  Udp.write(hdr, 2);
  Udp.write((const uint8_t*)tele_buf,
            (size_t)tele_count * sizeof(Sample));
  Udp.endPacket();
  tele_count = 0;
}

void push_sample(const Sample& s) {
  tele_buf[tele_count++] = s;
  if (tele_count >= MAX_SAMPLES_PER_PACKET) {
    flush_telemetry();
  }
}

void send_final_summary(const Sample& last, uint32_t total_cycles) {
  if (macPort == 0) return;
  uint8_t buf[1 + sizeof(Sample) + 4];
  buf[0] = TELE_FINAL;
  memcpy(buf + 1, &last, sizeof(Sample));
  memcpy(buf + 1 + sizeof(Sample), &total_cycles, 4);
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, sizeof(buf));
  Udp.endPacket();
}

// =====================================================================
// Motor sleep — safe shutdown
// =====================================================================
void motor_sleep_safe() {
  uint8_t tx[16], rx[32];
  size_t tx_len = build_motor_stream(tx, SUB_SLEEP, 0);
  modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 19);
}

// =====================================================================
// Motor connect (from teensy_orca_bridge)
// =====================================================================
bool motor_connect(uint32_t baud, uint16_t delay_us) {
  Serial1.end();
  Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);
  current_interframe_us = DEFAULT_INTERFRAME_US;
  delay(10);

  if (!verify_motor_present()) {
    send_info("motor not responding at default baud");
    return false;
  }
  send_info("motor verified at default baud");

  uint8_t tx[16], rx[16];
  size_t tx_len = build_manage_stream(tx, true, baud, delay_us);
  int n = modbus_transact(tx, tx_len, rx, sizeof(rx), 100000, 12);

  if (n != 12 || !check_crc(rx, n)
      || rx[0] != MOTOR_ADDR
      || rx[1] != FC_MANAGE_STREAM
      || rx[2] != 0xFF) {
    send_info("manage stream response invalid");
    return false;
  }

  uint32_t realized_baud  = ((uint32_t)rx[4] << 24) | ((uint32_t)rx[5] << 16)
                          | ((uint32_t)rx[6] << 8)  |  (uint32_t)rx[7];
  uint16_t realized_delay = ((uint16_t)rx[8] << 8) | rx[9];

  Serial1.end();
  Serial1.begin(realized_baud, SERIAL_8E1);
  current_interframe_us = realized_delay;
  delay(10);

  if (!verify_motor_present()) {
    send_info("motor not responding at new baud");
    Serial1.end();
    Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);
    current_interframe_us = DEFAULT_INTERFRAME_US;
    return false;
  }

  motor_connected = true;
  return true;
}

void motor_disconnect() {
  uint8_t tx[16], rx[16];
  size_t tx_len = build_manage_stream(tx, false, 0, 0);
  modbus_transact(tx, tx_len, rx, sizeof(rx), 100000, 12);
  Serial1.end();
  Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);
  current_interframe_us = DEFAULT_INTERFRAME_US;
  motor_connected = false;
}

// =====================================================================
// Autozero
//
// Writes config registers, sets motor mode to AutoZero, then polls
// until the motor leaves AutoZero mode or 6 s elapse.
// =====================================================================
bool motor_autozero() {
  uint8_t tx[16], rx[16];
  size_t tx_len;
  int n;

  // 1. Set zero mode = auto-zero enabled
  tx_len = build_write_single(tx, REG_ZERO_MODE, ZERO_MODE_AUTO_ZERO);
  n = modbus_transact(tx, tx_len, rx, sizeof(rx), 10000, 8);
  if (n != 8 || !check_crc(rx, n)) {
    send_info("autozero: write ZERO_MODE failed");
    return false;
  }

  // 2. Max force 50 N
  tx_len = build_write_single(tx, REG_AUTO_ZERO_FORCE_N, 50);
  modbus_transact(tx, tx_len, rx, sizeof(rx), 10000, 8);

  // 3. Max speed 100 mm/s
  tx_len = build_write_single(tx, REG_AUTO_ZERO_SPEED_MMPS, 100);
  modbus_transact(tx, tx_len, rx, sizeof(rx), 10000, 8);

  // 4. Exit to sleep mode after zeroing
  tx_len = build_write_single(tx, REG_AUTO_ZERO_EXIT_MODE, MOTOR_MODE_SLEEP);
  modbus_transact(tx, tx_len, rx, sizeof(rx), 10000, 8);

  // 5. Enter AutoZero mode (write to CTRL_REG_3, confirm via MODE_OF_OPERATION)
  tx_len = build_write_single(tx, REG_CTRL_REG_3, MOTOR_MODE_AUTO_ZERO);
  n = modbus_transact(tx, tx_len, rx, sizeof(rx), 10000, 8);
  if (n != 8 || !check_crc(rx, n)) {
    send_info("autozero: write CTRL_REG_3 failed");
    return false;
  }

  // 6. Poll MODE_OF_OPERATION until motor exits AutoZero mode (up to 6 s)
  elapsedMillis timer;
  while (timer < 6000) {
    delay(100);
    tx_len = build_read_holding(tx, REG_MODE_OF_OPERATION, 1);
    n = modbus_transact(tx, tx_len, rx, sizeof(rx), 10000, 7);
    // Response: addr(1) + fc(1) + bytecount(1) + 2 data + crc(2) = 7
    if (n == 7 && check_crc(rx, n) && rx[2] == 2) {
      uint16_t mode = ((uint16_t)rx[3] << 8) | rx[4];
      if (mode != MOTOR_MODE_AUTO_ZERO) {
        send_info("autozero complete");
        return true;
      }
    }

    // Check for errors
    tx_len = build_read_holding(tx, REG_ERROR_0, 1);
    n = modbus_transact(tx, tx_len, rx, sizeof(rx), 10000, 7);
    if (n == 7 && check_crc(rx, n) && rx[2] == 2) {
      uint16_t errs = ((uint16_t)rx[3] << 8) | rx[4];
      if (errs & 0x0080) {  // auto-zero failed bit
        send_info("autozero failed (motor error)");
        return false;
      }
    }
  }

  send_info("autozero timed out");
  return false;
}

// =====================================================================
// Physics: pendulum dynamics (RK4, 2-state, xddot held constant)
//
//   state = [theta, thetadot]
//   d/dt theta    = thetadot
//   d/dt thetadot = -(xddot * cos(theta) + g * sin(theta)) / l
// =====================================================================
static inline void pendulum_deriv(double theta, double thetadot,
                                  double xddot,
                                  double& d_theta, double& d_thetadot) {
  d_theta    = thetadot;
  d_thetadot = -(xddot * cos(theta) + par_g * sin(theta)) / par_l;
}

void rk4_pendulum(double h, double xddot) {
  double th = st_theta;
  double td = st_thetadot;

  double k1_th, k1_td;
  pendulum_deriv(th, td, xddot, k1_th, k1_td);

  double k2_th, k2_td;
  pendulum_deriv(th + 0.5*h*k1_th, td + 0.5*h*k1_td, xddot, k2_th, k2_td);

  double k3_th, k3_td;
  pendulum_deriv(th + 0.5*h*k2_th, td + 0.5*h*k2_td, xddot, k3_th, k3_td);

  double k4_th, k4_td;
  pendulum_deriv(th + h*k3_th, td + h*k3_td, xddot, k4_th, k4_td);

  double sixth = h / 6.0;
  st_theta    += sixth * (k1_th + 2.0*k2_th + 2.0*k3_th + k4_th);
  st_thetadot += sixth * (k1_td + 2.0*k2_td + 2.0*k3_td + k4_td);
}

// =====================================================================
// Force computation
//
// thetaddot = -(xddot * cos(theta) + g * sin(theta)) / l
// P         = m_p * l * (thetaddot * cos(theta) - thetadot^2 * sin(theta))
// f_pc      = -P
// f_command = (m_s / (m_c + m_p)) * f_pc
//
// Returns f_command in Newtons.  Also writes *out_fx and *out_fpc.
// =====================================================================
double compute_force(double theta, double thetadot, double xddot,
                     double* out_thetaddot,
                     double* out_fx, double* out_fpc) {
  double cs = cos(theta);
  double sn = sin(theta);

  double thetaddot = -(xddot * cs + par_g * sn) / par_l;
  *out_thetaddot = thetaddot;

  double P   = par_m_p * par_l * (thetaddot * cs - thetadot * thetadot * sn);
  double fpc = -P;
  *out_fpc = fpc;

  // Inferred human force (log only)
  *out_fx = (par_m_c + par_m_p) * xddot
          + par_m_p * par_l * thetaddot * cs
          - par_m_p * par_l * thetadot * thetadot * sn;

  double mass_ratio = par_m_s / (par_m_c + par_m_p);
  return mass_ratio * fpc;
}

// =====================================================================
// Sign self-check (MANDATORY before entering the loop)
//
// At theta=0, thetadot=0, xddot=+1:
//   thetaddot = -1/l  (pendulum begins swinging backward)
//   P = -m_p          (coupling term is negative)
//   f_pc = -P = +m_p  (positive: inertial reaction from the pendulum)
//   f_cmd > 0
//
// This is correct: at theta=0 the rod is vertical and the pendulum
// exerts no horizontal constraint force; the positive f_cmd reflects
// the inertial coupling in the EOM, not a restoring pull.
// =====================================================================
bool sign_check() {
  double thetaddot, fx, fpc;
  double f_cmd = compute_force(0.0, 0.0, 1.0,
                               &thetaddot, &fx, &fpc);

  char msg[128];
  snprintf(msg, sizeof(msg),
           "sign_check: thetaddot=%.6f fpc=%.6f f_cmd=%.6f fx=%.6f",
           thetaddot, fpc, f_cmd, fx);
  send_info(msg);

  return f_cmd > 0.0;
}

// =====================================================================
// Motor exchange — one extended-mode cycle
//
// 1. 0x64 force command (streams previous force, gets position+tele)
// 2. 0x03 read shaft speed
// 3. 0x03 read shaft acceleration
//
// Returns true if all three transactions succeeded.
// =====================================================================
MotorExchange motor_exchange(int32_t force_mN) {
  MotorExchange ex = {};
  uint8_t tx[16], rx[32];

  // Tx1: 0x64 force control
  size_t tx_len = build_motor_stream(tx, SUB_FORCE_CTRL, force_mN);
  int n1 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 19);
  bool ok_run = (n1 == 19) && parse_motor_stream_response(rx, n1);
  if (ok_run) {
    ex.position_um = motor_tele.position_um;
    ex.errors      = motor_tele.errors;
  }

  // Tx2: read shaft speed
  tx_len = build_read_holding(tx, REG_SHAFT_SPEED, 2);
  int n2 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 9);
  bool ok_speed = parse_read_holding_2reg_int32(rx, n2, &ex.speed_mmps);

  // Tx3: read shaft acceleration
  tx_len = build_read_holding(tx, REG_SHAFT_ACCEL, 2);
  int n3 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 9);
  bool ok_accel = parse_read_holding_2reg_int32(rx, n3, &ex.accel_mmpss);

  ex.t_meas_us = micros();
  ex.ok = ok_run && ok_speed && ok_accel;
  return ex;
}

// =====================================================================
// Main haptic loop
// =====================================================================
void haptic_loop() {
  double h_s = (double)par_loop_us * 1e-6;
  loop_cycle = 0;
  tele_count = 0;
  Sample last_sample = {};

  // --- Initial step: compute initial force before main loop ---
  // Use initial state with xddot = 0 to get the first f_pc
  double thetaddot_init, fx_init, fpc_init;
  double f_cmd_init = compute_force(st_theta, st_thetadot, 0.0,
                                    &thetaddot_init, &fx_init, &fpc_init);
  double f_cmd_mN_init = f_cmd_init * 1000.0;
  if (f_cmd_mN_init >  par_max_force_mN) f_cmd_mN_init =  par_max_force_mN;
  if (f_cmd_mN_init < -par_max_force_mN) f_cmd_mN_init = -par_max_force_mN;
  prev_force_mN = (int32_t)f_cmd_mN_init;

  send_ack(CMD_BEGIN);

  elapsedMicros loop_timer;
  bool running = true;

  while (running) {
    loop_timer = 0;

    // --- 1. Motor exchange: stream previous force, read kinematics ---
    MotorExchange ex = motor_exchange(prev_force_mN);

    if (!ex.ok) {
      motor_sleep_safe();
      send_error(ERR_MODBUS_TIMEOUT);
      flush_telemetry();
      return;
    }

    if (ex.errors != 0) {
      motor_sleep_safe();
      send_error_with_fault(ERR_MOTOR_FAULT, ex.errors);
      flush_telemetry();
      return;
    }

    // --- 2–3. Overwrite cart state from motor (SI units) ---
    st_x     = (double)ex.position_um * 1e-6;
    st_xdot  = (double)ex.speed_mmps  * 1e-3;
    st_xddot = (double)ex.accel_mmpss * 1e-3;

    // --- 4–6. Compute forces ---
    double thetaddot, fx, fpc;
    double f_cmd = compute_force(st_theta, st_thetadot, st_xddot,
                                 &thetaddot, &fx, &fpc);

    // --- 7. Convert to mN and clip ---
    double f_cmd_mN = f_cmd * 1000.0;
    if (f_cmd_mN >  par_max_force_mN) f_cmd_mN =  par_max_force_mN;
    if (f_cmd_mN < -par_max_force_mN) f_cmd_mN = -par_max_force_mN;
    prev_force_mN = (int32_t)f_cmd_mN;

    // --- 8. Integrate pendulum ---
    rk4_pendulum(h_s, st_xddot);

    // Check for NaN
    if (!isfinite(st_theta) || !isfinite(st_thetadot)) {
      motor_sleep_safe();
      send_error(ERR_NAN_STATE);
      flush_telemetry();
      return;
    }

    // --- 9–10. Record telemetry ---
    uint32_t elapsed = (uint32_t)loop_timer;

    Sample s;
    s.cycle        = loop_cycle;
    s.t_meas_us    = ex.t_meas_us;
    s.x            = (float)st_x;
    s.xdot         = (float)st_xdot;
    s.xddot        = (float)st_xddot;
    s.theta        = (float)st_theta;
    s.thetadot     = (float)st_thetadot;
    s.fx           = (float)fx;
    s.fpc          = (float)fpc;
    s.f_command_mN    = (float)f_cmd_mN;
    s.force_sensed_mN = (float)motor_tele.force_mn;
    s.loop_us         = elapsed;
    push_sample(s);
    last_sample = s;

    loop_cycle++;

    // --- 11. Check for UDP commands (non-blocking) ---
    int sz = Udp.parsePacket();
    if (sz > 0) {
      uint8_t pkt[64];
      int pn = Udp.read(pkt, sizeof(pkt));
      if (pn > 0) {
        macIP   = Udp.remoteIP();
        macPort = Udp.remotePort();
        if (pkt[0] == CMD_END || pkt[0] == CMD_SLEEP) {
          running = false;
        }
      }
    }

    // --- 12. Pace the loop ---
    while ((uint32_t)loop_timer < par_loop_us) {
      // spin
    }
  }

  // --- Clean shutdown ---
  motor_sleep_safe();
  flush_telemetry();
  send_final_summary(last_sample, loop_cycle);
  send_ack(CMD_END);
}

// =====================================================================
// Command handler (binary protocol)
// =====================================================================
void handle_command(const uint8_t* pkt, size_t len) {
  if (len == 0) return;

  macIP   = Udp.remoteIP();
  macPort = Udp.remotePort();

  uint8_t opcode = pkt[0];
  size_t  payload_len = len - 1;
  const uint8_t* payload = pkt + 1;

  switch (opcode) {

  case CMD_PING:
    if (payload_len != PAYLOAD_PING) { send_error(ERR_BADLEN); return; }
    send_ping_ack();
    break;

  case CMD_AUTOZERO: {
    if (payload_len != PAYLOAD_AUTOZERO) { send_error(ERR_BADLEN); return; }
    if (!motor_connected) {
      // Auto-connect at default baud for autozero
      if (!motor_connect(DEFAULT_BAUD, DEFAULT_INTERFRAME_US)) {
        send_error(ERR_MOTOR_FAULT);
        return;
      }
      phase = Phase::CONNECTED;
    }
    if (motor_autozero()) {
      send_ack(CMD_AUTOZERO);
    } else {
      send_error(ERR_MOTOR_FAULT);
    }
    break;
  }

  case CMD_CONFIG: {
    if (payload_len != PAYLOAD_CONFIG) { send_error(ERR_BADLEN); return; }
    float vals[7];
    memcpy(vals, payload, 28);

    par_m_c          = (double)vals[0];
    par_m_p          = (double)vals[1];
    par_m_s          = (double)vals[2];
    par_l            = (double)vals[3];
    par_g            = (double)vals[4];
    par_max_force_mN = (double)vals[5];
    par_loop_us      = (uint32_t)(vals[6] + 0.5f);

    configured = true;

    // Connect to motor if not already connected
    if (!motor_connected) {
      if (!motor_connect(DEFAULT_BAUD, DEFAULT_INTERFRAME_US)) {
        send_error(ERR_MOTOR_FAULT);
        return;
      }
    }
    phase = Phase::CONFIGURED;
    send_ack(CMD_CONFIG);
    break;
  }

  case CMD_INIT: {
    if (payload_len != PAYLOAD_INIT) { send_error(ERR_BADLEN); return; }
    float vals[4];
    memcpy(vals, payload, 16);
    st_x        = (double)vals[0];
    st_theta    = (double)vals[1];
    st_xdot     = (double)vals[2];
    st_thetadot = (double)vals[3];
    state_initialized = true;
    send_ack(CMD_INIT);
    break;
  }

  case CMD_BEGIN: {
    if (payload_len != PAYLOAD_BEGIN) { send_error(ERR_BADLEN); return; }

    // 1. Must be configured
    if (!configured) {
      send_error(ERR_NOT_CONFIGURED);
      return;
    }

    // 2. Check motor errors
    if (motor_tele.valid && motor_tele.errors != 0) {
      send_error_with_fault(ERR_MOTOR_FAULT, motor_tele.errors);
      return;
    }

    // 3. Sign self-check
    if (!sign_check()) {
      send_error(ERR_SIGN_CHECK);
      motor_sleep_safe();
      return;
    }
    send_info("sign check passed");

    // 4. Default initial state if not explicitly set
    if (!state_initialized) {
      st_x = 0.0; st_theta = 0.0;
      st_xdot = 0.0; st_thetadot = 0.0;
    }

    // 5–6. Enter haptic loop (sends ACK internally)
    phase = Phase::RUNNING;
    haptic_loop();
    phase = Phase::CONFIGURED;
    break;
  }

  case CMD_END:
    // If not in loop, just ack
    send_ack(CMD_END);
    break;

  case CMD_SLEEP:
    motor_sleep_safe();
    phase = Phase::IDLE;
    send_ack(CMD_SLEEP);
    break;

  default:
    send_error(ERR_BADOP);
    break;
  }
}

// =====================================================================
// Arduino entry points
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("teensy_cartpole_haptic starting");

  if (!Ethernet.begin(teensyIP, subnetMask, gateway)) {
    Serial.println("Ethernet.begin() failed");
    while (true) delay(1000);
  }
  Udp.begin(LOCAL_PORT);
  Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);

  Serial.print("Listening on UDP ");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.println(LOCAL_PORT);
}

void loop() {
  int sz = Udp.parsePacket();
  if (sz > 0) {
    uint8_t buf[64];
    int n = Udp.read(buf, sizeof(buf));
    if (n > 0) {
      handle_command(buf, n);
    }
  }
}
