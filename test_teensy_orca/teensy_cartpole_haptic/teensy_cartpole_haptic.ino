/*
 * teensy_cartpole_haptic.ino
 *
 * Integrated haptic cart-pole loop on Teensy 4.1.
 * 
 * Implements two main task variants of an MWorks cart-pendulum experiment:
 * 1) A human drives the motor shaft ("cart"); the Teensy
 * reads shaft kinematics, integrates the virtual pendulum via RK4,
 * and commands the motor to render the pendulum reaction force.
 * 
 * 2) A human drives the motor shaft to a particular position (center-out-reach).
 * There are no pendulum dynamics in this task variant
 *
 * Mac sends binary UDP commands to the teensy (see TEENSY_API.md).
 * Under valid codes, teensy executes commands to an Orca motor over Modbus RTU.
 * Teensy streams packed binary telemetry back.
 *
 * Hardware: Teensy 4.1 + Ethernet kit (QNEthernet),
 *           MAX490 RS-422 on Serial1 (pins 0 RX, 1 TX),
 *           IrisDynamics Orca motor on Modbus RTU.
 */

#include <QNEthernet.h>
#include <math.h>
#include "stiction_model.h"   // Part A export: stiction_lookup_mN (boost_stiction, Task 0)
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
// Modbus configuration
// =====================================================================
constexpr uint8_t  MOTOR_ADDR            = 0x01;
constexpr uint32_t DEFAULT_BAUD          = 1000000; //must match motor's settings
constexpr uint32_t DEFAULT_INTERFRAME_US = 20; //must match motor's settings
constexpr uint32_t USER_COMMS_TIMEOUT_US = 500000; //500 milliseconds

constexpr uint8_t FC_READ_HOLDING        = 0x03;
constexpr uint8_t FC_WRITE_SINGLE        = 0x06;
constexpr uint8_t FC_WRITE_MULTIPLE      = 0x10;
constexpr uint8_t FC_MANAGE_STREAM       = 0x41;
constexpr uint8_t FC_MOTOR_CMD_STREAM    = 0x64;

constexpr uint8_t SUB_SLEEP              = 0x00;
constexpr uint8_t SUB_FORCE_CTRL         = 0x1C;  // FORCE_CMD register 28 (0x1C)
constexpr uint8_t SUB_POSITION_CTRL      = 0x1E;  // POS_CMD   register 30 (0x1E); used by MOVE_TO

// MOVE_TO: after the quintic trajectory finishes, keep streaming the setpoint
// for up to this long so the motor's internal position PID can close any
// remaining following error before we judge success. Exits early once within
// the error threshold; failure is reported only if still outside it after this.
// may need to tune the Position Controller fo the motor
constexpr uint32_t MOVE_SETTLE_US        = 500000;  // 0.5 s settle buffer

constexpr uint16_t REG_SHAFT_SPEED       = 344;
constexpr uint16_t REG_SHAFT_ACCEL       = 346;

// Autozero registers (from Orca SDK / Reference Manual)
// Verify these against RM220115 Table 2 if autozero misbehaves.
constexpr uint16_t REG_ZERO_MODE            = 171;
constexpr uint16_t REG_AUTO_ZERO_FORCE_N    = 172;
constexpr uint16_t REG_AUTO_ZERO_SPEED_MMPS = 177;
constexpr uint16_t REG_AUTO_ZERO_EXIT_MODE  = 173;
constexpr uint16_t REG_CTRL_REG_3           = 3;
constexpr uint16_t REG_MODE_OF_OPERATION    = 317;  // read-only: confirms current mode
constexpr uint16_t REG_ERROR_0              = 432;

constexpr uint16_t ZERO_MODE_AUTO_ZERO      = 2;
constexpr uint16_t MOTOR_MODE_SLEEP         = 1;
constexpr uint16_t MOTOR_MODE_AUTO_ZERO     = 55;

// =====================================================================
// Binary protocol — opcodes (Mac → Teensy)
// =====================================================================
constexpr uint8_t CMD_PING     = 0x01;
constexpr uint8_t CMD_AUTOZERO = 0x02; // entero autozero of motor
constexpr uint8_t CMD_CONFIG   = 0x03; // configure parameters of cart-pendulum system
constexpr uint8_t CMD_INIT_STATE   = 0x04;  // set initial pendulum/cart kinematic state (haptic)
constexpr uint8_t CMD_ENTER_HAPTIC = 0x05;  // enter the haptic loop (Task 0)
// 0x06 reserved (was CMD_END — folded into CMD_EXIT_MODE)
constexpr uint8_t CMD_SLEEP    = 0x07;
constexpr uint8_t CMD_ENTER_COR = 0x08;  // enter center-out-reach mode (force, base 0, no physics)
constexpr uint8_t CMD_MOVE_TO   = 0x09;  // quintic position move to a setpoint
constexpr uint8_t CMD_EXIT_MODE = 0x0A;  // leave the active loop (haptic or COR) → motor sleep / idle
constexpr uint8_t CMD_CALIBRATE_HANDLE = 0x0B;  // 2 s handle baseline capture
// 0x0C–0x0F reserved for Tasks 3/4 → ERR_BADOP.

// Reply opcodes (Teensy → Mac)
constexpr uint8_t REPLY_ACK     = 0xA1;
constexpr uint8_t REPLY_ERROR   = 0xA2;
constexpr uint8_t TELE_BATCH    = 0xB0;  // full haptic Sample batch (Task 0)
constexpr uint8_t TELE_FINAL    = 0xB1;
constexpr uint8_t TELE_BATCH_COR = 0xB2;  // reduced SampleCOR batch (Tasks 1, 2)

// Error codes
constexpr uint8_t ERR_BADOP           = 0x01;
constexpr uint8_t ERR_BADLEN          = 0x02;
constexpr uint8_t ERR_MOTOR_FAULT     = 0x03;
constexpr uint8_t ERR_SIGN_CHECK      = 0x04;
constexpr uint8_t ERR_NAN_STATE       = 0x05;
constexpr uint8_t ERR_MODBUS_TIMEOUT  = 0x06;
constexpr uint8_t ERR_NOT_CONFIGURED  = 0x07;
constexpr uint8_t ERR_MOVE_FAILED     = 0x08;  // MOVE_TO ended outside threshold or faulted
constexpr uint8_t ERR_HANDLE_CAL      = 0x09;  // handle baseline implausible (wide band / rail-pinned)
constexpr uint8_t ERR_ANGLE_LIMIT     = 0x0A;  // pendulum exceeded the safety angle (runaway swing) — loop aborted

// Expected payload sizes per opcode
constexpr size_t PAYLOAD_PING      = 0;
constexpr size_t PAYLOAD_AUTOZERO  = 0;
constexpr size_t PAYLOAD_CONFIG    = 52;  // 13 × float (7 physics/loop + 6 boost_stiction)
constexpr size_t PAYLOAD_INIT_STATE   = 16;  // 4 × float
constexpr size_t PAYLOAD_ENTER_HAPTIC = 0;
constexpr size_t PAYLOAD_SLEEP_CMD = 0;
constexpr size_t PAYLOAD_ENTER_COR = 0;
constexpr size_t PAYLOAD_MOVE_TO   = 16;  // 4 × float: setpoint_um, duration_s, loop_period_us, error_threshold_um
constexpr size_t PAYLOAD_EXIT_MODE = 0;
constexpr size_t PAYLOAD_CALIBRATE_HANDLE = 20;  // 5 × float boost params

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
// boost_stiction — Task 0 haptic loop only.
//
// Compensates the shaft's physical static friction so the rendered dynamics stay
// faithful as the shaft slows into the stiction regime. Magnitude comes from the
// compiled-in GP breakaway table (stiction_model.h): the FULL looked-up F_static
// for |v| <= par_stiction_v_thresh_mmps, then exponentially decaying with speed
// scale par_stiction_decay_mmps above it (the "stiction_only" envelope formulated
// in Arduino/validate_controller). Direction = sign(f_command) at/below the
// threshold (intended push at rest), sign(v) above (friction opposes motion).
// All five fields arrive in CMD_CONFIG; defaults below keep the boost OFF until a
// session configures it, so Task 0 is unchanged when disabled.
//
// Combine mode folds boost_stiction together with the handle boost_human.
constexpr uint8_t COMBINE_ADDITIVE_CAPPED = 0;  // sum, clip combined magnitude to cap
constexpr uint8_t COMBINE_HUMAN_OVERRIDE  = 1;  // human alone when out of deadband, else stiction
constexpr uint8_t COMBINE_MAX_ALIGNED     = 2;  // same sign -> larger magnitude; opposite -> sum

bool    par_stiction_enable      = false;
double  par_stiction_mult        = 1.0;
double  par_stiction_v_thresh_mmps = 350.0;
double  par_stiction_decay_mmps    = 100.0;
uint8_t par_boost_combine_mode   = COMBINE_ADDITIVE_CAPPED;
double  par_boost_cap_mN         = 50000.0;

// Safety abort (haptic loop only): if the pendulum swings past this angle from
// hanging (theta=0 is straight down), the loop stops, sleeps the motor, and
// reports ERR_ANGLE_LIMIT. 120 deg = horizontal (90) + 30, i.e. heading toward
// inversion/runaway. Checked on the wrapped angle so a continuing spin also trips.
constexpr double MAX_PENDULUM_ANGLE_RAD = 120.0 * M_PI / 180.0;  // 2.0944 rad

// =====================================================================
// Force-sensing handle — boost_human
//
// The handle is an AD620-amplified Wheatstone bridge read on A0
// Its resting voltage drifts session-to-session, so we capture
// a fresh [baseline_min, baseline_max] band with CMD_CALIBRATE_HANDLE before any
// loop runs. boost_human_mN() maps voltage outside that band to a signed assist
// force; inside the band it is exactly 0. Not velocity-gated. Until a successful
// calibration, handle_calibrated stays false and the boost is forced to 0, so
// the COR loop behaves exactly as it did before the handle existed.
// =====================================================================
constexpr int   HANDLE_PIN   = A0;       // matches wheatstone.ino FORCE_PIN
constexpr float ADC_VREF     = 3.3f;     // Teensy 4.1 analog reference (V)
constexpr float ADC_FULLSCALE = 4095.0f; // 12-bit range 0..4095

// boost_human shape (mN). par_boost_min at the ramp edge (±DV past the band),
// asymptoting to par_boost_max at the rail. These are bench-tunable
// the Mac sends fresh values in the CMD_CALIBRATE_HANDLE payload,
// so editing them in teensy_common.mwel and recalibrating updates the feel without reflashing.
// par_boost_dv / par_boost_k are floored on apply to avoid div-by-zero.
float par_boost_min_mN = 12000.0f; // 12 N at the ramp edge
float par_boost_max_mN = 15000.0f; // plateau toward this at the rail
float par_boost_dv     = 0.20f;    // onset margin past the band (V)
float par_boost_k      = 3.0f;     // exponential ramp shape
float par_boost_m      = 4.0f;     // saturation curvature toward the rail

// Calibration bounds: reject an implausible baseline.
constexpr float HANDLE_CAL_SECONDS   = 2.0f;
constexpr float HANDLE_CAL_MAX_BAND  = 0.40f;  // resting band wider than this → reject
constexpr float HANDLE_CAL_RAIL_LO   = 0.30f;  // min below this → pinned low → reject
constexpr float HANDLE_CAL_RAIL_HI   = 3.00f;  // max above this → pinned high → reject

float handle_baseline_min = 0.0f;
float handle_baseline_max = 0.0f;
bool  handle_calibrated   = false;

// Handle voltage EMA: the raw ADC read is noisy and a rapid grip change can jump
// the boost by tens of N in one cycle (bench-observed). A first-order EMA on the
// voltage feeding boost_human_mN() removes that buzz. Calibration still samples
// the RAW voltage (it must capture the true resting noise band). Re-seeded at the
// start of each loop so there is no startup transient from 0.
constexpr float HANDLE_EMA_ALPHA = 0.1f;   // ~15 Hz cutoff at a 1200 us loop
float handle_v_filt      = 0.0f;
bool  handle_v_filt_init = false;

// A single corrupted/short RS-422 frame should NOT kill the whole run, and we
// can't afford to retry
// Instead, hold the last good reading: the motor still
// received our 0x64 force command (the TX went out fine; only the readback frame
// glitched), so the physics does its forward pass with the previously read value
// for whichever field failed and we move on with no added latency. A *genuine*
// comms break is reported by the motor itself as COMMS_TIMEOUT (error bit 2048)
// in the next valid frame, which the ex.errors check in the loop catches and
// aborts on. These hold the most recent valid reading of each field; reset at
// the start of haptic_loop().
int32_t prev_position_um = 0;
int32_t prev_speed_mmps  = 0;
int32_t prev_accel_mmpss = 0;
uint32_t modbus_glitch_count = 0;  // single-frame glitches held-over this run (diagnostic)

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
// The five force-boost fields are appended after the haptic fields and before
// loop_us. All now carry signal in Task 0: handle_voltage / boost_human_mN from
// the handle, boost_stiction_mN / stiction_gate from the gated GP breakaway
// (0 when stiction_enable is false), and boost_combined_mN = the post-combine
// boost actually applied (combine_boosts()).
//
// Python struct format: '<II14fI'
// Total: 4+4 + 14*4 + 4 = 68 bytes
// =====================================================================
struct __attribute__((packed)) Sample {
  uint32_t cycle;
  uint32_t t_meas_us;
  float    x, xdot, xddot;
  float    theta, thetadot;
  float    fx;
  float    fpc;
  float    f_command_mN;
  float    force_sensed_mN;   // motor_tele.force_mn (lags ~3 frames)
  float    handle_voltage;    // EMA-smoothed handle ADC voltage this cycle (V)
  float    boost_human_mN;    // boost_human before clip (signed mN)
  float    boost_stiction_mN; // gated stiction boost before clip (signed mN); 0 when disabled
  float    stiction_gate;     // stiction velocity envelope 0..1; 0 when disabled
  float    boost_combined_mN; // combined boost added before clip (combine_boosts)
  uint32_t loop_us;
};
static_assert(sizeof(Sample) == 68, "Sample struct packing");

constexpr int MAX_SAMPLES_PER_PACKET = 8;
Sample   tele_buf[MAX_SAMPLES_PER_PACKET];
uint8_t  tele_count = 0;

// =====================================================================
// Reduced telemetry for COR mode (Tasks 1, 2) — the haptic Sample with
// the pendulum-only fields (theta, thetadot, fx, fpc, force_sensed_mN)
// removed, plus the handle force-boost fields appended so the
// boost_human mapping can be verified on the bench. Distinct discriminator byte
// (TELE_BATCH_COR = 0xB2).
//
// Python struct format: '<II6fI'   Total: 4+4 + 6*4 + 4 = 36 bytes
// =====================================================================
struct __attribute__((packed)) SampleCOR {
  uint32_t cycle;
  uint32_t t_meas_us;     // micros() at the moment xddot was measured
  float    x, xdot, xddot;// measured shaft state (SI)
  float    f_command_mN;  // commanded force after boost + safety clip
  float    handle_voltage;// raw handle ADC voltage this cycle (V)
  float    boost_human_mN;// boost_human before the safety clip (signed mN)
  uint32_t loop_us;
};
static_assert(sizeof(SampleCOR) == 36, "SampleCOR struct packing");

SampleCOR cor_tele_buf[MAX_SAMPLES_PER_PACKET];
uint8_t   cor_tele_count = 0;

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
// CRC-16 Modbus
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
// Serial / Modbus I/O
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
// Modbus frame builders
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
// Motor verification
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

// --- COR telemetry batching (leading byte TELE_BATCH_COR = 0xB2) ---
void flush_telemetry_cor() {
  if (cor_tele_count == 0 || macPort == 0) return;
  uint8_t hdr[2] = { TELE_BATCH_COR, cor_tele_count };
  Udp.beginPacket(macIP, macPort);
  Udp.write(hdr, 2);
  Udp.write((const uint8_t*)cor_tele_buf,
            (size_t)cor_tele_count * sizeof(SampleCOR));
  Udp.endPacket();
  cor_tele_count = 0;
}

void push_sample_cor(const SampleCOR& s) {
  cor_tele_buf[cor_tele_count++] = s;
  if (cor_tele_count >= MAX_SAMPLES_PER_PACKET) {
    flush_telemetry_cor();
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
// Motor connect
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

  // Each transaction is a single attempt — no retry, so a glitch costs zero
  // extra latency. On a bad/short frame we keep the previous good value for that
  // field (held in prev_*) and carry on.

  // Tx1: 0x64 force control + position/error readback.
  size_t tx_len = build_motor_stream(tx, SUB_FORCE_CTRL, force_mN);
  // Small per-transaction gap timeout (5 ms): a glitched/missing reply must bail
  // fast so hold-last-value adds no loop latency. NOT the motor's COMMS_TIMEOUT
  // watchdog, which is a separate register.
  int n1 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 19);
  if ((n1 == 19) && parse_motor_stream_response(rx, n1)) {
    prev_position_um = motor_tele.position_um;
    ex.errors        = motor_tele.errors;
  } else {
    modbus_glitch_count++;
  }
  ex.position_um = prev_position_um;

  // Tx2: read shaft speed
  tx_len = build_read_holding(tx, REG_SHAFT_SPEED, 2);
  int n2 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 9);
  int32_t speed_mmps;
  if (parse_read_holding_2reg_int32(rx, n2, &speed_mmps)) {
    prev_speed_mmps = speed_mmps;
  } else {
    modbus_glitch_count++;
  }
  ex.speed_mmps = prev_speed_mmps;

  // Tx3: read shaft acceleration
  tx_len = build_read_holding(tx, REG_SHAFT_ACCEL, 2);
  int n3 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 9);
  int32_t accel_mmpss;
  if (parse_read_holding_2reg_int32(rx, n3, &accel_mmpss)) {
    prev_accel_mmpss = accel_mmpss;
  } else {
    modbus_glitch_count++;
  }
  ex.accel_mmpss = prev_accel_mmpss;

  ex.t_meas_us = micros();
  // A single-frame glitch is non-fatal (handled via hold-last-value above). The
  // only abort condition is a motor-reported fault in ex.errors (e.g.
  // COMMS_TIMEOUT 2048 on a genuine comms break), checked by the caller.
  ex.ok = true;
  return ex;
}

// =====================================================================
// Force-sensing handle: read, boost mapping, calibration (Step 3 Part C)
// =====================================================================

// One ADC read per control cycle. 12-bit range 0..4095
float read_handle_voltage() {
  int raw = analogRead(HANDLE_PIN);
  return (float)raw * (ADC_VREF / ADC_FULLSCALE);
}

// EMA-smoothed handle voltage for the control loops. Seeds on the first call of a
// run (handle_v_filt_init reset at loop entry) so there is no transient from 0.
// This is what feeds boost_human_mN(); calibration uses the raw read instead.
float read_handle_voltage_filtered() {
  float v = read_handle_voltage();
  if (!handle_v_filt_init) {
    handle_v_filt = v;
    handle_v_filt_init = true;
  } else {
    handle_v_filt += HANDLE_EMA_ALPHA * (v - handle_v_filt);
  }
  return handle_v_filt;
}

// boost_human: handle voltage → signed assist force (mN). Zero inside the
// measured deadband; exponential ramp 0→±BOOST_MIN over DV past the band edge;
// then asymptotic plateau toward ±BOOST_MAX at the rail (3.3 V right / 0 V left).
// Continuous, monotonic, sign = push direction. Depends ONLY on voltage (not
// velocity-gated). Returns 0 until a successful CMD_CALIBRATE_HANDLE.
float boost_human_mN(float v) {
  if (!handle_calibrated) return 0.0f;

  const float b_lo = handle_baseline_min;
  const float b_hi = handle_baseline_max;

  if (v >= b_lo && v <= b_hi) return 0.0f;   // deadband

  if (v > b_hi) {
    // Rightward (push right → position increases): positive boost.
    if (v <= b_hi + par_boost_dv) {
      float u = (v - b_hi) / par_boost_dv;                   // 0..1
      return par_boost_min_mN * (expf(par_boost_k * u) - 1.0f) / (expf(par_boost_k) - 1.0f);
    }
    float denom = ADC_VREF - (b_hi + par_boost_dv);
    float w = (denom > 1e-6f) ? (v - (b_hi + par_boost_dv)) / denom : 1.0f;
    if (w > 1.0f) w = 1.0f;
    float f = par_boost_max_mN - (par_boost_max_mN - par_boost_min_mN) * expf(-par_boost_m * w);
    return (f > par_boost_max_mN) ? par_boost_max_mN : f;
  } else {
    // Leftward (push left → position decreases): mirror image, negative boost.
    if (v >= b_lo - par_boost_dv) {
      float u = (b_lo - v) / par_boost_dv;                   // 0..1
      return -par_boost_min_mN * (expf(par_boost_k * u) - 1.0f) / (expf(par_boost_k) - 1.0f);
    }
    float denom = b_lo - par_boost_dv;                       // band edge → 0 V rail
    float w = (denom > 1e-6f) ? ((b_lo - par_boost_dv) - v) / denom : 1.0f;
    if (w > 1.0f) w = 1.0f;
    float f = par_boost_max_mN - (par_boost_max_mN - par_boost_min_mN) * expf(-par_boost_m * w);
    return -((f > par_boost_max_mN) ? par_boost_max_mN : f);
  }
}

// Sample the resting handle for HANDLE_CAL_SECONDS, returning the measured
// min/max voltage. Result valid only if the band is narrow and off the rails.
bool calibrate_handle(float* out_min, float* out_max) {
  float vmin = 1e9f, vmax = -1e9f;
  elapsedMicros t = 0;
  uint32_t window_us = (uint32_t)(HANDLE_CAL_SECONDS * 1e6f);
  while ((uint32_t)t < window_us) {
    float v = read_handle_voltage();
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
    delayMicroseconds(500);  // ~4000 samples over 2 s
  }
  *out_min = vmin;
  *out_max = vmax;
  if ((vmax - vmin) > HANDLE_CAL_MAX_BAND) return false;  // implausibly wide
  if (vmin < HANDLE_CAL_RAIL_LO) return false;            // pinned low
  if (vmax > HANDLE_CAL_RAIL_HI) return false;            // pinned high
  return true;
}

// boost_stiction: compiled-in GP breakaway table, applied at FULL strength
// for |v| <= par_stiction_v_thresh_mmps, then exponentially decaying with speed
// scale par_stiction_decay_mmps above it. Direction follows the intended command
// (sign of f_dyn_mN) at/below the threshold — no motion sign yet — and the actual
// motion (sign of v) above it, since sliding friction opposes velocity. Writes
// the envelope it used into *out_gate (0..1). Returns 0 / gate 0 when disabled.
float boost_stiction_mN(double f_dyn_mN, double v_mmps, int32_t pos_um,
                        float* out_gate) {
  *out_gate = 0.0f;
  if (!par_stiction_enable) return 0.0f;

  double abs_v = fabs(v_mmps);
  double env;
  int    dir;
  if (abs_v <= par_stiction_v_thresh_mmps) {
    env = 1.0;                              // full F_static near rest
    dir = (f_dyn_mN >= 0.0) ? +1 : -1;      // directed by the intended command
  } else {
    env = (par_stiction_decay_mmps > 0.0)
          ? exp(-(abs_v - par_stiction_v_thresh_mmps) / par_stiction_decay_mmps)
          : 0.0;                            // <=0 -> hard cutoff above the threshold
    dir = (v_mmps >= 0.0) ? +1 : -1;        // directed by actual motion (opposes v)
  }
  *out_gate = (float)env;
  // Table is signed (positive extend / negative retract), so dir already carries
  // the sign; mult scales it, env applies the velocity envelope.
  return stiction_lookup_mN((float)pos_um, dir)
         * (float)par_stiction_mult * (float)env;
}

// Combine the handle boost_human with boost_stiction per the runtime mode (C.5).
// Both are signed mN (same sign = reinforce, opposite = partial cancel).
float combine_boosts(float boost_human, float boost_stiction) {
  switch (par_boost_combine_mode) {
    case COMBINE_HUMAN_OVERRIDE:
      // Human actively pushing (out of deadband) -> use it alone, else stiction.
      return (boost_human != 0.0f) ? boost_human : boost_stiction;
    case COMBINE_MAX_ALIGNED: {
      bool same_sign = (boost_human >= 0.0f) == (boost_stiction >= 0.0f);
      if (same_sign)  // friction is overcome once -> take the larger magnitude
        return (fabsf(boost_human) >= fabsf(boost_stiction)) ? boost_human
                                                             : boost_stiction;
      return boost_human + boost_stiction;  // opposite signs -> sum (partial cancel)
    }
    case COMBINE_ADDITIVE_CAPPED:
    default: {
      float c = boost_human + boost_stiction;
      if (c >  (float)par_boost_cap_mN) c =  (float)par_boost_cap_mN;
      if (c < -(float)par_boost_cap_mN) c = -(float)par_boost_cap_mN;
      return c;
    }
  }
}

// ACK/ERROR reply carrying the measured baseline (two little-endian floats), so
// the Mac logs the band per session even when calibration fails.
void send_handle_cal_reply(uint8_t tag, uint8_t code, float vmin, float vmax) {
  if (macPort == 0) return;
  uint8_t buf[10];
  buf[0] = tag;
  buf[1] = code;
  memcpy(buf + 2, &vmin, 4);  // little-endian on ARM
  memcpy(buf + 6, &vmax, 4);
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, 10);
  Udp.endPacket();
}

// =====================================================================
// Main haptic loop
// =====================================================================
void haptic_loop() {
  double h_s = (double)par_loop_us * 1e-6;
  loop_cycle = 0;
  tele_count = 0;
  modbus_glitch_count = 0;
  prev_position_um = 0;
  prev_speed_mmps  = 0;
  prev_accel_mmpss = 0;
  handle_v_filt_init = false;   // re-seed the handle EMA on the first cycle
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

  send_ack(CMD_ENTER_HAPTIC);

  elapsedMicros loop_timer;
  bool    running  = true;
  uint8_t exit_op  = CMD_EXIT_MODE;  // opcode to ack on exit (mirrors cor_loop)

  // loop_us reports the *previous* iteration's elapsed work time (steps 1–11,
  // including the UDP-command poll) since that duration isn't known until after
  // the current sample has already been pushed. First sample reports 0.
  uint32_t prev_loop_us = 0;

  while (running) {
    loop_timer = 0;

    // --- 1. Motor exchange: stream previous force, read kinematics ---
    // A glitched readback frame is non-fatal: motor_exchange() holds the last
    // good value for that field so the physics still steps forward this cycle.
    // The only abort is a motor-reported fault below — COMMS_TIMEOUT (2048)
    // surfaces there on a genuine comms break.
    MotorExchange ex = motor_exchange(prev_force_mN);

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

    // --- 7. Convert to mN, add the boosts (combined), then clip ---
    // Two assist terms, both added before the safety clip:
    //   boost_human    — handle ADC mapped to a signed assist force (deadband -> 0)
    //   boost_stiction — gated GP breakaway compensating the shaft's static
    //                    friction (sign keyed to the pre-boost dynamics command at
    //                    rest, to motion while sliding). 0 unless configured.
    // combine_boosts() folds them per par_boost_combine_mode. f_dyn_mN below
    // is the pure dynamics command (pre-boost) — its sign is the intended push, so
    // it drives the at-rest stiction direction. With the handle uncalibrated AND
    // stiction disabled, both boosts are 0 and Task 0 behaves exactly as before.
    double f_dyn_mN = f_cmd * 1000.0;
    float  v_handle       = read_handle_voltage_filtered();   // EMA-smoothed
    float  boost_h_mN     = boost_human_mN(v_handle);
    float  stiction_gate  = 0.0f;
    float  boost_stict_mN = boost_stiction_mN(f_dyn_mN, (double)ex.speed_mmps,
                                              ex.position_um, &stiction_gate);
    float  boost_comb_mN  = combine_boosts(boost_h_mN, boost_stict_mN);
    double f_cmd_mN = f_dyn_mN + (double)boost_comb_mN;
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
    s.handle_voltage    = v_handle;
    s.boost_human_mN    = boost_h_mN;
    s.boost_stiction_mN = boost_stict_mN;
    s.stiction_gate     = stiction_gate;
    s.boost_combined_mN = boost_comb_mN;
    s.loop_us         = prev_loop_us;  // previous iteration's duration (incl. step 11)
    push_sample(s);
    last_sample = s;

    loop_cycle++;

    // --- Safety: abort if the pendulum swings past the angle limit ---
    // Wrap to (-pi, pi] so a continuing spin (theta growing past pi) also trips.
    // The tripping sample is already logged above; sleep the motor and report.
    double theta_wrapped = atan2(sin(st_theta), cos(st_theta));
    if (fabs(theta_wrapped) > MAX_PENDULUM_ANGLE_RAD) {
      motor_sleep_safe();
      send_error(ERR_ANGLE_LIMIT);
      flush_telemetry();
      return;
    }

    // --- 11. Check for UDP commands (non-blocking) ---
    int sz = Udp.parsePacket();
    if (sz > 0) {
      uint8_t pkt[64];
      int pn = Udp.read(pkt, sizeof(pkt));
      if (pn > 0) {
        macIP   = Udp.remoteIP();
        macPort = Udp.remotePort();
        if (pkt[0] == CMD_EXIT_MODE || pkt[0] == CMD_SLEEP) {
          running = false;
          exit_op = pkt[0];
        } else if (pkt[0] == CMD_PING) {
          // Answer PING inline so the Mac's end-of-session clock_sync() works
          // while the loop is still streaming. Cheap and rare (only the ~11
          // PINGs of a clock-sync burst, ~10 ms apart). This is the timebase
          // the telemetry t_meas_us is stamped in, so it's the most useful
          // anchor for post-hoc alignment.
          send_ping_ack();
        }
      }
    }

    // Elapsed work time for this iteration (steps 1–11), reported by the
    // next iteration's sample. Captured before the pacing spin so it reflects
    // real work, not the fixed loop period.
    prev_loop_us = (uint32_t)loop_timer;

    // --- 12. Pace the loop ---
    while ((uint32_t)loop_timer < par_loop_us) {
      // spin
    }
  }

  // --- Clean shutdown ---
  motor_sleep_safe();
  flush_telemetry();
  {
    char msg[64];
    snprintf(msg, sizeof(msg), "modbus glitches held-over: %lu",
             (unsigned long)modbus_glitch_count);
    send_info(msg);
  }
  send_final_summary(last_sample, loop_cycle);
  send_ack(exit_op);
}

// =====================================================================
// COR loop (Tasks 1, 2) — center-out-reach mode.
//
// Same motor exchange as the haptic loop (force stream + speed + accel
// reads), but the commanded force is a fixed base of 0 (transparent shaft)
// and there is NO pendulum integration. Streams reduced SampleCOR records
// (leading byte TELE_BATCH_COR = 0xB2). Exits on EXIT_MODE / END / SLEEP and
// acks the opcode that triggered the exit.
// =====================================================================
void cor_loop() {
  loop_cycle = 0;
  cor_tele_count = 0;
  modbus_glitch_count = 0;
  prev_position_um = 0;
  prev_speed_mmps  = 0;
  prev_accel_mmpss = 0;
  handle_v_filt_init = false;   // re-seed the handle EMA on the first cycle

  // Commanded force = boost_human, computed fresh each cycle from
  // the handle voltage. Until a session runs CMD_CALIBRATE_HANDLE the boost is
  // forced to 0, so this is a transparent (base-0) shaft exactly as before.
  // The Task 0 phase will add the velocity-gated boost_stiction term on top.

  send_ack(CMD_ENTER_COR);

  elapsedMicros loop_timer;
  bool     running   = true;
  uint8_t  exit_op   = CMD_EXIT_MODE;  // opcode to ack on exit
  uint32_t prev_loop_us = 0;

  while (running) {
    loop_timer = 0;

    // --- 0. Handle boost: read voltage, map to a signed assist force, clip ---
    // boost_human is independent of the motor exchange (just an ADC read), so it
    // is computed before streaming. Added before the safety clip; in the
    // Task 1 phase "the boost" is just boost_human (no stiction term yet).
    float v_handle   = read_handle_voltage_filtered();   // EMA-smoothed
    float boost_mN   = boost_human_mN(v_handle);
    double cmd_mN    = (double)boost_mN;
    if (cmd_mN >  par_max_force_mN) cmd_mN =  par_max_force_mN;
    if (cmd_mN < -par_max_force_mN) cmd_mN = -par_max_force_mN;
    int32_t cmd_force_mN = (int32_t)cmd_mN;

    // --- 1. Motor exchange: stream the boost force, read kinematics ---
    MotorExchange ex = motor_exchange(cmd_force_mN);
    if (ex.errors != 0) {
      motor_sleep_safe();
      send_error_with_fault(ERR_MOTOR_FAULT, ex.errors);
      flush_telemetry_cor();
      return;
    }

    // --- 2. Overwrite shaft state from motor (SI). No physics step. ---
    st_x     = (double)ex.position_um * 1e-6;
    st_xdot  = (double)ex.speed_mmps  * 1e-3;
    st_xddot = (double)ex.accel_mmpss * 1e-3;

    // --- 3. Record reduced telemetry ---
    SampleCOR s;
    s.cycle          = loop_cycle;
    s.t_meas_us      = ex.t_meas_us;
    s.x              = (float)st_x;
    s.xdot           = (float)st_xdot;
    s.xddot          = (float)st_xddot;
    s.f_command_mN   = (float)cmd_mN;
    s.handle_voltage = v_handle;
    s.boost_human_mN = boost_mN;
    s.loop_us        = prev_loop_us;
    push_sample_cor(s);

    loop_cycle++;

    // --- 4. Check for UDP commands (non-blocking) ---
    int sz = Udp.parsePacket();
    if (sz > 0) {
      uint8_t pkt[64];
      int pn = Udp.read(pkt, sizeof(pkt));
      if (pn > 0) {
        macIP   = Udp.remoteIP();
        macPort = Udp.remotePort();
        if (pkt[0] == CMD_EXIT_MODE || pkt[0] == CMD_SLEEP) {
          running = false;
          exit_op = pkt[0];
        } else if (pkt[0] == CMD_PING) {
          send_ping_ack();  // inline so end-of-session clock_sync works mid-stream
        }
      }
    }

    prev_loop_us = (uint32_t)loop_timer;

    // --- 5. Pace the loop ---
    while ((uint32_t)loop_timer < par_loop_us) {
      // spin
    }
  }

  // --- Clean shutdown: sleep motor (transparent handoff done), flush, ack ---
  motor_sleep_safe();
  flush_telemetry_cor();
  send_ack(exit_op);
}

// =====================================================================
// Position-aware reply helpers (MOVE_TO). The achieved final position is
// appended as a little-endian int32 (µm) for Mac-side logging.
// =====================================================================
void send_ack_with_pos(uint8_t cmd_opcode, int32_t pos_um) {
  if (macPort == 0) return;
  uint8_t buf[6];
  buf[0] = REPLY_ACK;
  buf[1] = cmd_opcode;
  memcpy(buf + 2, &pos_um, 4);  // little-endian on ARM
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, 6);
  Udp.endPacket();
}

void send_error_with_pos(uint8_t err_code, int32_t pos_um) {
  if (macPort == 0) return;
  uint8_t buf[6];
  buf[0] = REPLY_ERROR;
  buf[1] = err_code;
  memcpy(buf + 2, &pos_um, 4);
  Udp.beginPacket(macIP, macPort);
  Udp.write(buf, 6);
  Udp.endPacket();
}

// =====================================================================
// MOVE_TO — quintic minimum-jerk position move.
//
// Drives the motor in Position Mode (stream sub-code SUB_POSITION_CTRL = 0x1E)
// along a minimum-jerk profile from the current shaft position to setpoint_um
// over duration_s, sampled at loop_period_us. Used to home the shaft to
// stroke-centre at trial start. On completion replies ACK 0x09 + int32
// final_pos when |final - setpoint| <= error_threshold_um, else ERROR
// ERR_MOVE_FAILED + int32 final_pos. On motor fault, sleeps and reports
// ERR_MOVE_FAILED. Leaves the motor holding the last position command (no
// sleep) on success so a following CMD_ENTER_COR hands off promptly.
// =====================================================================
void do_move_to(float setpoint_um_f, float duration_s, float loop_period_us_f,
                float error_threshold_um_f) {
  // Read current position as the trajectory start (one force-0 exchange).
  prev_position_um = 0;
  prev_speed_mmps  = 0;
  prev_accel_mmpss = 0;
  MotorExchange ex0 = motor_exchange(0);
  if (ex0.errors != 0) {
    motor_sleep_safe();
    send_error_with_pos(ERR_MOVE_FAILED, ex0.position_um);
    return;
  }

  int32_t start_um    = ex0.position_um;
  int32_t setpoint_um = (int32_t)lroundf(setpoint_um_f);
  double  span_um     = (double)setpoint_um - (double)start_um;

  uint32_t loop_us = (uint32_t)(loop_period_us_f + 0.5f);
  if (loop_us < 200) loop_us = 200;       // floor; avoid runaway / div-by-zero
  double T = (double)duration_s;
  if (T < 1e-3) T = 1e-3;
  uint32_t nsteps = (uint32_t)(T * 1e6 / (double)loop_us);
  if (nsteps < 1) nsteps = 1;

  int32_t last_pos_um = start_um;
  elapsedMicros step_timer;

  for (uint32_t i = 1; i <= nsteps; i++) {
    step_timer = 0;

    double tau = (double)i / (double)nsteps;                      // 0..1
    double s   = tau*tau*tau*(10.0 - 15.0*tau + 6.0*tau*tau);     // 10t^3-15t^4+6t^5
    int32_t pos_um = start_um + (int32_t)lround(span_um * s);

    // Stream the position command; read back position + errors.
    uint8_t tx[16], rx[32];
    size_t tx_len = build_motor_stream(tx, SUB_POSITION_CTRL, pos_um);
    int n = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 19);
    if (n == 19 && parse_motor_stream_response(rx, n)) {
      last_pos_um = motor_tele.position_um;
      if (motor_tele.errors != 0) {
        motor_sleep_safe();
        send_error_with_pos(ERR_MOVE_FAILED, last_pos_um);
        return;
      }
    } else {
      modbus_glitch_count++;  // single glitched readback is non-fatal; hold last_pos_um
    }

    // Pace to loop_us.
    while ((uint32_t)step_timer < loop_us) {
      // spin
    }
  }

  // Resolve the success threshold (default 100 um) up front — the settle phase
  // below uses it for early termination.
  int32_t threshold = (int32_t)lroundf(error_threshold_um_f);
  if (threshold <= 0) threshold = 100;  // default 100 um

  // ---- Settle phase: hold the setpoint, let the motor's PID converge --------
  // The quintic ends with a zero-velocity command at setpoint, but for an
  // aggressive/long span the position-PID loop can still have a following error
  // at trajectory end. Keep streaming the setpoint for up to MOVE_SETTLE_US,
  // polling the readback each loop period, and exit the moment we are within
  // threshold. If the buffer elapses still outside threshold, the final check
  // below reports ERR_MOVE_FAILED.
  {
    elapsedMicros settle_timer = 0;
    while ((uint32_t)settle_timer < MOVE_SETTLE_US) {
      step_timer = 0;

      uint8_t tx[16], rx[32];
      size_t tx_len = build_motor_stream(tx, SUB_POSITION_CTRL, setpoint_um);
      int n = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 19);
      if (n == 19 && parse_motor_stream_response(rx, n)) {
        last_pos_um = motor_tele.position_um;
        if (motor_tele.errors != 0) {
          motor_sleep_safe();
          send_error_with_pos(ERR_MOVE_FAILED, last_pos_um);
          return;
        }
        int32_t e = last_pos_um - setpoint_um;
        if (e < 0) e = -e;
        if (e <= threshold) break;   // converged → report success
      } else {
        modbus_glitch_count++;       // glitched readback non-fatal; hold last_pos_um
      }

      // Pace to loop_us.
      while ((uint32_t)step_timer < loop_us) {
        // spin
      }
    }
  }

  // Final position check (after the trajectory + settle phase).
  int32_t err = last_pos_um - setpoint_um;
  if (err < 0) err = -err;
  if (err <= threshold) {
    send_ack_with_pos(CMD_MOVE_TO, last_pos_um);
  } else {
    send_error_with_pos(ERR_MOVE_FAILED, last_pos_um);
  }
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
    float vals[13];
    memcpy(vals, payload, 52);

    par_m_c          = (double)vals[0];
    par_m_p          = (double)vals[1];
    par_m_s          = (double)vals[2];
    par_l            = (double)vals[3];
    par_g            = (double)vals[4];
    par_max_force_mN = (double)vals[5];
    par_loop_us      = (uint32_t)(vals[6] + 0.5f);

    // boost_stiction params (Step 3 Part C). Task 0 haptic loop only; the COR
    // tasks ignore them. Edits in teensy_common.mwel land here each reset().
    par_stiction_enable        = (vals[7]  > 0.5f);
    par_stiction_mult          = (double)vals[8];
    par_stiction_v_thresh_mmps = (double)vals[9];
    par_stiction_decay_mmps    = (double)vals[10];
    par_boost_combine_mode     = (uint8_t)(vals[11] + 0.5f);
    par_boost_cap_mN           = (double)vals[12];

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

  case CMD_INIT_STATE: {
    if (payload_len != PAYLOAD_INIT_STATE) { send_error(ERR_BADLEN); return; }
    float vals[4];
    memcpy(vals, payload, 16);
    st_x        = (double)vals[0];
    st_theta    = (double)vals[1];
    st_xdot     = (double)vals[2];
    st_thetadot = (double)vals[3];
    state_initialized = true;
    send_ack(CMD_INIT_STATE);
    break;
  }

  case CMD_ENTER_HAPTIC: {
    if (payload_len != PAYLOAD_ENTER_HAPTIC) { send_error(ERR_BADLEN); return; }

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

  case CMD_SLEEP:
    motor_sleep_safe();
    phase = Phase::IDLE;
    send_ack(CMD_SLEEP);
    break;

  case CMD_ENTER_COR: {
    if (payload_len != PAYLOAD_ENTER_COR) { send_error(ERR_BADLEN); return; }
    // Motor must be connected (via CMD_CONFIG or CMD_AUTOZERO) to run the exchange.
    if (!motor_connected) { send_error(ERR_NOT_CONFIGURED); return; }
    phase = Phase::RUNNING;
    cor_loop();              // blocks until EXIT_MODE/END/SLEEP; acks internally
    phase = Phase::CONFIGURED;
    break;
  }

  case CMD_MOVE_TO: {
    if (payload_len != PAYLOAD_MOVE_TO) { send_error(ERR_BADLEN); return; }
    if (!motor_connected) { send_error(ERR_NOT_CONFIGURED); return; }
    float vals[4];
    memcpy(vals, payload, 16);  // setpoint_um, duration_s, loop_period_us, error_threshold_um
    do_move_to(vals[0], vals[1], vals[2], vals[3]);   // replies internally
    break;
  }

  case CMD_EXIT_MODE:
    // Inside haptic_loop()/cor_loop()/do_move_to() this is caught by their UDP
    // poll (handled there). Received at idle, just ack so the Mac's
    // exit_haptic()/exit_cor() doesn't time out.
    if (payload_len != PAYLOAD_EXIT_MODE) { send_error(ERR_BADLEN); return; }
    send_ack(CMD_EXIT_MODE);
    break;

  case CMD_CALIBRATE_HANDLE: {
    if (payload_len != PAYLOAD_CALIBRATE_HANDLE) { send_error(ERR_BADLEN); return; }
    // Adopt the boost shape params shipped with this calibration (so a tweak in
    // teensy_common.mwel takes effect on the next recalibration, never mid-run).
    float bvals[5];
    memcpy(bvals, payload, 20);  // boost_min_mN, boost_max_mN, boost_dv, boost_k, boost_m
    par_boost_min_mN = bvals[0];
    par_boost_max_mN = bvals[1];
    par_boost_dv     = bvals[2];
    par_boost_k      = bvals[3];
    par_boost_m      = bvals[4];
    if (par_boost_dv < 1e-3f) par_boost_dv = 1e-3f;  // floor: avoid div-by-zero
    if (par_boost_k  < 1e-3f) par_boost_k  = 1e-3f;  // floor: ramp denom expf(k)-1
    // 2 s resting-baseline capture (Step 3 C.0). Reply carries the measured band
    // (two floats) on both success and failure so the Mac logs it per session.
    float vmin, vmax;
    if (calibrate_handle(&vmin, &vmax)) {
      handle_baseline_min = vmin;
      handle_baseline_max = vmax;
      handle_calibrated   = true;
      send_handle_cal_reply(REPLY_ACK, CMD_CALIBRATE_HANDLE, vmin, vmax);
    } else {
      handle_calibrated = false;  // boost stays 0 until a good baseline is captured
      send_handle_cal_reply(REPLY_ERROR, ERR_HANDLE_CAL, vmin, vmax);
    }
    break;
  }

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

  // Force-sensing handle ADC (Step 3 Part C): 12-bit, no hardware averaging
  // (matches wheatstone.ino — the boost deadband + ramp do the smoothing).
  analogReadResolution(12);
  analogReadAveraging(1);

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
