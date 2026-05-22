/*
 * teensy_orca_bridge.ino
 *
 * Hybrid architecture:
 *   - Mac (Python) sends high-level commands to Teensy over UDP.
 *   - Teensy autonomously streams Modbus 0x64 frames to the ORCA motor.
 *   - Teensy continuously reports telemetry + errors back to Mac over UDP.
 *
 * Hardware:
 *   - Teensy 4.1 with ethernet kit (QNEthernet)
 *   - MAX490 RS-422 transceiver on Serial1 (pins 0 RX, 1 TX)
 *   - Motor connected via MAX490 to ORCA RJ45 (see header comments in
 *     the conversation for wiring details)
 */

#include <QNEthernet.h>
using namespace qindesign::network;

// =====================================================================
// Network configuration
// =====================================================================
IPAddress teensyIP   (192, 168, 1, 177);
IPAddress subnetMask (255, 255, 255, 0);
IPAddress gateway    (192, 168, 1, 1);

constexpr unsigned int LOCAL_PORT       = 8888; // Teensy listens here
constexpr unsigned int TELEMETRY_PORT   = 8889; // Mac listens here

EthernetUDP Udp;
IPAddress macIP;          // Learned from first received packet
uint16_t  macPort = 0;

// =====================================================================
// Modbus configuration
// =====================================================================
constexpr uint8_t  MOTOR_ADDR             = 0x01;
constexpr uint32_t DEFAULT_BAUD           = 1000000;
constexpr uint32_t DEFAULT_INTERFRAME_US  = 0;

// Function codes
constexpr uint8_t FC_READ_HOLDING         = 0x03;
constexpr uint8_t FC_WRITE_SINGLE         = 0x06;
constexpr uint8_t FC_WRITE_MULTIPLE       = 0x10;
constexpr uint8_t FC_MANAGE_STREAM        = 0x41;
constexpr uint8_t FC_MOTOR_CMD_STREAM     = 0x64;

// 0x64 sub-codes
constexpr uint8_t SUB_SLEEP               = 0x00;
constexpr uint8_t SUB_FORCE_CTRL          = 0x1C;
constexpr uint8_t SUB_POSITION_CTRL       = 0x1E;

// =====================================================================
// State machine
// =====================================================================
enum class Phase : uint8_t {
  IDLE,
  CONNECT,
  STREAM_ENABLED,
  STREAMING,
  DISCONNECTING,
  DISCONNECTED
};
Phase phase = Phase::IDLE;

bool streaming        = false;   // 0x64 frames active?
uint8_t  stream_sub   = SUB_SLEEP;
int32_t  stream_data  = 0;       // Force in mN, position in µm, etc.

uint32_t stream_period_us  = 0;  // stream as fast as possible
uint32_t telemetry_period_us = 10000; // Report to Mac at 100 Hz default
elapsedMicros since_last_stream;
elapsedMicros since_last_telemetry;

// Extended mode: each cycle does 0x64 motor.run() + read shaft speed
// + read acceleration. Per-transaction times are recorded so we can
// characterize the control loop floor.
bool extended_mode = false;
constexpr uint16_t REG_SHAFT_SPEED = 344;   // mm/s, double-wide signed
constexpr uint16_t REG_SHAFT_ACCEL = 346;   // mm/s^2, double-wide signed

// Latest extended telemetry (filled in extended mode)
struct ExtTelemetry {
  // From 0x64 response
  int32_t  position_um;
  int32_t  force_mn;
  uint16_t power_w;
  int8_t   temperature_c;
  uint16_t voltage_mv;
  uint16_t errors;
  // From 0x03 reads
  int32_t  shaft_speed_mmps;
  int32_t  shaft_accel_mmpss;
  // Per-transaction timing in microseconds
  uint32_t t_run_us;
  uint32_t t_speed_us;
  uint32_t t_accel_us;
  uint32_t t_total_us;
  // Cycle sequence number — Python uses this to detect drops
  uint32_t seq;
  bool     valid;
};
ExtTelemetry ext = {};

// Latest telemetry parsed from motor (Stream Command Response PDU, Table 6)
struct MotorTelemetry {
  int32_t  position_um;
  int32_t  force_mn;
  uint16_t power_w;
  int8_t   temperature_c;
  uint16_t voltage_mv;
  uint16_t errors;
  bool     valid;
};
MotorTelemetry tele = {};

// =====================================================================
// CRC-16 Modbus (polynomial 0xA001), per page 15 of the User Guide
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

// Append CRC (low byte first, per Modbus RTU) and return total length.
size_t append_crc(uint8_t* buf, size_t len) {
  uint16_t crc = modbus_crc(buf, len);
  buf[len]     = crc & 0xFF;        // CRC low
  buf[len + 1] = (crc >> 8) & 0xFF; // CRC high
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
uint32_t current_interframe_us = DEFAULT_INTERFRAME_US;

// Wait for the configured interframe gap before sending the next frame.
void interframe_delay() {
  delayMicroseconds(current_interframe_us);
}

// Send a Modbus frame and read the response. Returns as soon as
// expected_len bytes have arrived, or when timeout_us elapses since the
// last received byte (whichever comes first).
//
// Pass expected_len = 0 if you don't know the response size in advance —
// the function will then fall back to byte-gap timeout, but that wastes
// timeout_us at the end of every transaction. For Modbus, you almost
// always know the length, so pass it.
int modbus_transact(const uint8_t* tx, size_t tx_len,
                    uint8_t* rx, size_t rx_max,
                    uint32_t timeout_us,
                    size_t expected_len = 0) {
  interframe_delay();

  // Flush any stale RX bytes.
  while (Serial1.available()) Serial1.read();

  Serial1.write(tx, tx_len);
  Serial1.flush();

  elapsedMicros t;
  size_t got = 0;
  size_t target = (expected_len > 0 && expected_len <= rx_max) ? expected_len : rx_max;
  while (t < timeout_us && got < target) {
    if (Serial1.available()) {
      rx[got++] = Serial1.read();
      t = 0;  // restart byte-gap timer
      if (got >= target) break;
    }
  }
  return got;
}

// =====================================================================
// Build Modbus frames
// =====================================================================

// 0x06 Write Single Register
size_t build_write_single(uint8_t* buf, uint16_t addr, uint16_t value) {
  buf[0] = MOTOR_ADDR;
  buf[1] = FC_WRITE_SINGLE;
  buf[2] = (addr >> 8) & 0xFF;
  buf[3] = addr & 0xFF;
  buf[4] = (value >> 8) & 0xFF;
  buf[5] = value & 0xFF;
  return append_crc(buf, 6);
}

// 0x03 Read Holding Registers
size_t build_read_holding(uint8_t* buf, uint16_t addr, uint16_t count) {
  buf[0] = MOTOR_ADDR;
  buf[1] = FC_READ_HOLDING;
  buf[2] = (addr >> 8) & 0xFF;
  buf[3] = addr & 0xFF;
  buf[4] = (count >> 8) & 0xFF;
  buf[5] = count & 0xFF;
  return append_crc(buf, 6);
}

// Parse a Read Holding Registers response that returns exactly 2 registers
// (4 data bytes) encoded as a little-endian 32-bit value per the user
// guide convention "high register address holds the upper bytes".
// Expected response: addr(1) + fc(1) + bytecount(1) + 4 data + crc(2) = 9 bytes.
//   data[0..1] = register at addr   (low word)
//   data[2..3] = register at addr+1 (high word)
// Result = (high << 16) | low, interpreted as int32_t.
bool parse_read_holding_2reg_int32(const uint8_t* buf, size_t len, int32_t* out) {
  if (len != 9)                   return false;
  if (buf[0] != MOTOR_ADDR)       return false;
  if (buf[1] != FC_READ_HOLDING)  return false;
  if (buf[2] != 4)                return false;
  if (!check_crc(buf, len))       return false;
  uint16_t lo = ((uint16_t)buf[3] << 8) | buf[4];
  uint16_t hi = ((uint16_t)buf[5] << 8) | buf[6];
  uint32_t v  = ((uint32_t)hi << 16) | lo;
  *out = (int32_t)v;
  return true;
}

// 0x41 Manage High-Speed Stream
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

// 0x64 Motor Command Stream (Force / Position / Sleep)
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

// =====================================================================
// Parse 0x64 Motor Command Stream Response (Table 6, page 7)
// =====================================================================
bool parse_motor_stream_response(const uint8_t* buf, size_t len) {
  // Expected: addr(1) + fc(1) + pos(4) + force(4) + power(2) + temp(1)
  //         + voltage(2) + errors(2) + crc(2) = 19 bytes
  if (len != 19) return false;
  if (buf[0] != MOTOR_ADDR) return false;
  if (buf[1] != FC_MOTOR_CMD_STREAM) return false;
  if (!check_crc(buf, len)) return false;

  tele.position_um    = ((int32_t)buf[2]  << 24) | ((int32_t)buf[3]  << 16)
                      | ((int32_t)buf[4]  << 8)  |  (int32_t)buf[5];
  tele.force_mn       = ((int32_t)buf[6]  << 24) | ((int32_t)buf[7]  << 16)
                      | ((int32_t)buf[8]  << 8)  |  (int32_t)buf[9];
  tele.power_w        = ((uint16_t)buf[10] << 8) | buf[11];
  tele.temperature_c  = (int8_t)buf[12];
  tele.voltage_mv     = ((uint16_t)buf[13] << 8) | buf[14];
  tele.errors         = ((uint16_t)buf[15] << 8) | buf[16];
  tele.valid          = true;
  return true;
}

// =====================================================================
// Motor presence verification (used as a handshake during CONNECT)
//
// Reads the Serial Number registers (406, 407) per page 17 of the user
// guide. A correctly-framed response with valid CRC proves:
//   - the serial wiring is correct
//   - the motor is powered and responding
//   - the current baud/parity match the motor's settings
//   - our CRC implementation matches the motor's
// =====================================================================
bool verify_motor_present() {
  uint8_t tx[8];
  tx[0] = MOTOR_ADDR;
  tx[1] = FC_READ_HOLDING;
  tx[2] = 0x01; tx[3] = 0x96;  // register address 406
  tx[4] = 0x00; tx[5] = 0x02;  // read 2 registers
  size_t tx_len = append_crc(tx, 6);

  uint8_t rx[32];
  int n = modbus_transact(tx, tx_len, rx, sizeof(rx), 50000, 9);

  // Diagnostic: report what we actually received so we can tell whether
  // the motor is silent, garbled, or returning a Modbus exception.
  char dbg[256];
  if (n == 0) {
    snprintf(dbg, sizeof(dbg), "INFO verify_rx: 0 bytes (motor silent)");
  } else {
    int off = snprintf(dbg, sizeof(dbg), "INFO verify_rx: %d bytes:", n);
    for (int i = 0; i < n && off < (int)sizeof(dbg) - 4; i++) {
      off += snprintf(dbg + off, sizeof(dbg) - off, " %02X", rx[i]);
    }
  }
  send_udp(dbg);

  // Expected response: addr(1) + fc(1) + byte_count(1) + 4 data bytes + crc(2) = 9 bytes
  if (n != 9)                     return false;
  if (rx[0] != MOTOR_ADDR)        return false;
  if (rx[1] != FC_READ_HOLDING)   return false;
  if (rx[2] != 4)                 return false;
  if (!check_crc(rx, n))          return false;
  return true;
}

// =====================================================================
// UDP message protocol (text, simple to debug)
// =====================================================================
// Commands from Mac → Teensy:
//   CONNECT <baud> <delay_us>
//   ENABLE_STREAM <sub_code_hex> <data> <period_us>
//       e.g. "ENABLE_STREAM 1C 0 1000" for sleep stream at 1 kHz
//       e.g. "ENABLE_STREAM 1C 1000 1000" for force=1000mN at 1 kHz
//   SET <data>             update stream data (e.g. new force target)
//   SLEEP                  switch active stream to Sleep sub-code (0x00) —
//                          motor stops generating forces but stream stays
//                          alive (avoids comms-timeout error 2048).
//                          Call before DISABLE_STREAM for clean exit.
//   DISABLE_STREAM
//   DISCONNECT
//   PING
//
// Messages Teensy → Mac:
//   TELEMETRY <phase> <pos_um> <force_mn> <power_w> <temp_c> <voltage_mv> <errors>
//   ERROR <phase> <message>
//   ACK <command>
// =====================================================================

const char* phase_name(Phase p) {
  switch (p) {
    case Phase::IDLE:           return "IDLE";
    case Phase::CONNECT:        return "CONNECT";
    case Phase::STREAM_ENABLED: return "STREAM_ENABLED";
    case Phase::STREAMING:      return "STREAMING";
    case Phase::DISCONNECTING:  return "DISCONNECTING";
    case Phase::DISCONNECTED:   return "DISCONNECTED";
  }
  return "UNKNOWN";
}

void send_udp(const char* msg) {
  if (macPort == 0) return; // No Mac to send to yet
  Udp.beginPacket(macIP, TELEMETRY_PORT);
  Udp.write(msg);
  Udp.endPacket();
}

void send_error(const char* msg) {
  char buf[160];
  snprintf(buf, sizeof(buf), "ERROR %s %s", phase_name(phase), msg);
  send_udp(buf);
}

void send_ack(const char* cmd) {
  char buf[64];
  snprintf(buf, sizeof(buf), "ACK %s", cmd);
  send_udp(buf);
}

void send_telemetry() {
  char buf[200];
  snprintf(buf, sizeof(buf),
    "TELEMETRY %s %ld %ld %u %d %u %u",
    phase_name(phase),
    (long)tele.position_um,
    (long)tele.force_mn,
    (unsigned)tele.power_w,
    (int)tele.temperature_c,
    (unsigned)tele.voltage_mv,
    (unsigned)tele.errors);
  send_udp(buf);
}

// Extended-mode telemetry: one packet per cycle, including per-transaction
// timing. Format:
//   EXT_TELEMETRY <seq> <t_run_us> <t_speed_us> <t_accel_us> <t_total_us>
//                 <pos_um> <force_mn> <power_w> <temp_c> <voltage_mv> <errors>
//                 <speed_mmps> <accel_mmpss>
void send_ext_telemetry() {
  char buf[256];
  snprintf(buf, sizeof(buf),
    "EXT_TELEMETRY %lu %lu %lu %lu %lu %ld %ld %u %d %u %u %ld %ld",
    (unsigned long)ext.seq,
    (unsigned long)ext.t_run_us,
    (unsigned long)ext.t_speed_us,
    (unsigned long)ext.t_accel_us,
    (unsigned long)ext.t_total_us,
    (long)ext.position_um,
    (long)ext.force_mn,
    (unsigned)ext.power_w,
    (int)ext.temperature_c,
    (unsigned)ext.voltage_mv,
    (unsigned)ext.errors,
    (long)ext.shaft_speed_mmps,
    (long)ext.shaft_accel_mmpss);
  send_udp(buf);
}

// =====================================================================
// Command handlers
// =====================================================================

void cmd_connect(uint32_t baud, uint16_t delay_us) {
  phase = Phase::CONNECT;

  // -----------------------------------------------------------------
  // Phase 1: Open Serial1 at DEFAULT baud and verify motor is present.
  // If the motor doesn't answer here, there's no point trying the
  // baud switch — bail with a clear error.
  // -----------------------------------------------------------------
  Serial1.end();
  Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);
  current_interframe_us = DEFAULT_INTERFRAME_US;
  delay(10);

  if (!verify_motor_present()) {
    send_error("MOTOR_NOT_RESPONDING_AT_DEFAULT_BAUD");
    phase = Phase::IDLE;
    return;
  }
  send_udp("INFO motor_verified_at_default_baud");

  // -----------------------------------------------------------------
  // Phase 2: Send 0x41 to switch to high-speed stream baud/delay.
  // Validate the response is a proper echo with the right function
  // code and state byte, not just any 12 bytes that happen to CRC.
  // -----------------------------------------------------------------
  uint8_t tx[16];
  uint8_t rx[16];
  size_t tx_len = build_manage_stream(tx, true, baud, delay_us);
  int n = modbus_transact(tx, tx_len, rx, sizeof(rx), 100000, 12);

  if (n != 12 || !check_crc(rx, n)
      || rx[0] != MOTOR_ADDR
      || rx[1] != FC_MANAGE_STREAM
      || rx[2] != 0xFF) {
    send_error("MANAGE_STREAM_RESPONSE_INVALID");
    phase = Phase::IDLE;
    return;
  }

  // Report the realized baud/delay — the motor may not honor the exact
  // request (e.g. it picks the closest achievable baud).
  // Per the example frame on page 16, the response echoes the full
  // request (12 bytes), so baud occupies rx[4..7] and delay rx[8..9],
  // NOT rx[3..6] / rx[7..8] as Table 4 implies.
  uint32_t realized_baud  = ((uint32_t)rx[4] << 24) | ((uint32_t)rx[5] << 16)
                          | ((uint32_t)rx[6] << 8)  |  (uint32_t)rx[7];
  uint16_t realized_delay = ((uint16_t)rx[8] << 8) | rx[9];

  char msg[96];
  snprintf(msg, sizeof(msg), "INFO realized_baud=%lu delay_us=%u",
           (unsigned long)realized_baud, (unsigned)realized_delay);
  send_udp(msg);

  // -----------------------------------------------------------------
  // Phase 3: Reconfigure Serial1 to the realized baud and verify
  // we can still talk to the motor at the new settings.
  // If this fails, fall back to default so the motor isn't left
  // stranded at a baud rate we can't reach.
  // -----------------------------------------------------------------
  Serial1.end();
  Serial1.begin(realized_baud, SERIAL_8E1);
  current_interframe_us = realized_delay;
  delay(10);

  if (!verify_motor_present()) {
    send_error("MOTOR_NOT_RESPONDING_AT_NEW_BAUD");
    Serial1.end();
    Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);
    current_interframe_us = DEFAULT_INTERFRAME_US;
    phase = Phase::IDLE;
    return;
  }

  send_ack("CONNECT");
}

void cmd_enable_stream(uint8_t sub_code, int32_t data, uint32_t period_us) {
  if (phase != Phase::CONNECT && phase != Phase::STREAM_ENABLED) {
    send_error("ENABLE_REQUIRES_CONNECT");
    return;
  }
  stream_sub   = sub_code;
  stream_data  = data;
  stream_period_us = period_us;
  streaming    = true;
  extended_mode = false;
  phase        = Phase::STREAMING;
  since_last_stream = stream_period_us; // fire immediately
  send_ack("ENABLE_STREAM");
}

// Like ENABLE_STREAM but each cycle does three Modbus transactions:
//   1. 0x64 motor.run() — sends current force/position/sleep command
//   2. 0x03 read shaft speed (32-bit, mm/s)
//   3. 0x03 read shaft acceleration (32-bit, mm/s^2)
// Sub-transaction timing is recorded per cycle and reported in EXT_TELEMETRY.
// stream_period_us = 0 means "as fast as possible" — each cycle starts
// immediately after the previous completes.
void cmd_enable_extended_stream(uint8_t sub_code, int32_t data, uint32_t period_us) {
  if (phase != Phase::CONNECT && phase != Phase::STREAM_ENABLED) {
    send_error("ENABLE_REQUIRES_CONNECT");
    return;
  }
  stream_sub   = sub_code;
  stream_data  = data;
  stream_period_us = period_us;
  streaming    = true;
  extended_mode = true;
  ext.seq      = 0;
  phase        = Phase::STREAMING;
  since_last_stream = stream_period_us; // fire immediately
  send_ack("ENABLE_EXT_STREAM");
}

void cmd_set(int32_t data) {
  // Update the streamed data on the fly (e.g. new force target).
  stream_data = data;
  // No ACK on this hot path to keep latency low; client can rely on telemetry.
}

// Switch the active stream to Sleep mode (sub-code 0x00). The motor will
// stop generating forces but the stream stays alive, so the communication
// timeout (page 14 of the user guide) is NOT triggered. Call this before
// DISABLE_STREAM / DISCONNECT to leave the motor in a clean state.
void cmd_sleep() {
  stream_sub  = SUB_SLEEP;
  stream_data = 0;
  // If we weren't streaming, we still want sleep frames flowing for a
  // moment so the motor exits Force/Position mode cleanly.
  if (!streaming) {
    streaming = true;
    phase     = Phase::STREAMING;
    since_last_stream = stream_period_us; // fire immediately
  }
  send_ack("SLEEP");
}

void cmd_disable_stream() {
  streaming = false;
  extended_mode = false;
  phase = Phase::STREAM_ENABLED;
  send_ack("DISABLE_STREAM");
}

void cmd_disconnect() {
  phase = Phase::DISCONNECTING;
  streaming = false;

  // Send 0x41 disable — motor returns to default baud.
  uint8_t tx[16];
  uint8_t rx[16];
  size_t tx_len = build_manage_stream(tx, false, 0, 0);
  modbus_transact(tx, tx_len, rx, sizeof(rx), 100000, 12);

  // Whether or not it acked, drop our serial back to default.
  Serial1.end();
  Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);
  current_interframe_us = DEFAULT_INTERFRAME_US;

  phase = Phase::DISCONNECTED;
  send_ack("DISCONNECT");
}

// Parse incoming UDP command line and dispatch.
void handle_command(char* line) {
  // Remember sender so telemetry has somewhere to go.
  macIP = Udp.remoteIP();
  macPort = Udp.remotePort();

  char* tok = strtok(line, " \r\n");
  if (!tok) return;

  if      (strcmp(tok, "CONNECT") == 0) {
    char* a = strtok(NULL, " \r\n");
    char* b = strtok(NULL, " \r\n");
    if (a && b) cmd_connect((uint32_t)atol(a), (uint16_t)atoi(b));
    else        send_error("CONNECT_BAD_ARGS");
  }
  else if (strcmp(tok, "ENABLE_STREAM") == 0) {
    char* a = strtok(NULL, " \r\n");
    char* b = strtok(NULL, " \r\n");
    char* c = strtok(NULL, " \r\n");
    if (a && b && c) {
      uint8_t sub = (uint8_t)strtol(a, NULL, 16);
      cmd_enable_stream(sub, (int32_t)atol(b), (uint32_t)atol(c));
    } else send_error("ENABLE_BAD_ARGS");
  }
  else if (strcmp(tok, "ENABLE_EXT_STREAM") == 0) {
    char* a = strtok(NULL, " \r\n");
    char* b = strtok(NULL, " \r\n");
    char* c = strtok(NULL, " \r\n");
    if (a && b && c) {
      uint8_t sub = (uint8_t)strtol(a, NULL, 16);
      cmd_enable_extended_stream(sub, (int32_t)atol(b), (uint32_t)atol(c));
    } else send_error("ENABLE_EXT_BAD_ARGS");
  }
  else if (strcmp(tok, "SET") == 0) {
    char* a = strtok(NULL, " \r\n");
    if (a) cmd_set((int32_t)atol(a));
  }
  else if (strcmp(tok, "DISABLE_STREAM") == 0) cmd_disable_stream();
  else if (strcmp(tok, "SLEEP")          == 0) cmd_sleep();
  else if (strcmp(tok, "DISCONNECT")     == 0) cmd_disconnect();
  else if (strcmp(tok, "PING")           == 0) send_ack("PING");
  else send_error("UNKNOWN_COMMAND");
}

// =====================================================================
// Setup / Loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("teensy_orca_bridge starting");

  if (!Ethernet.begin(teensyIP, subnetMask, gateway)) {
    Serial.println("ERROR: Ethernet.begin() failed.");
    while (true) { delay(1000); }
  }
  Udp.begin(LOCAL_PORT);

  // Start Serial1 at the default; cmd_connect will switch as needed.
  Serial1.begin(DEFAULT_BAUD, SERIAL_8E1);

  Serial.print("Listening on UDP ");
  Serial.print(Ethernet.localIP());
  Serial.print(":");
  Serial.println(LOCAL_PORT);
}

void loop() {
  // ---- Receive any UDP command ----
  int sz = Udp.parsePacket();
  if (sz > 0) {
    char buf[256];
    int n = Udp.read(buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = 0;
      handle_command(buf);
    }
  }

  // ---- Send next Modbus stream frame(s) on schedule ----
  if (streaming && since_last_stream >= stream_period_us) {
    since_last_stream = 0;

    uint8_t tx[16];
    uint8_t rx[32];

    if (extended_mode) {
      // === EXTENDED MODE: 0x64 + read speed + read accel, per cycle ===
      elapsedMicros cycle_t;

      // ---- Tx1: 0x64 motor.run() ----
      elapsedMicros t1;
      size_t tx_len = build_motor_stream(tx, stream_sub, stream_data);
      int n1 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 19);
      ext.t_run_us = (uint32_t)t1;
      bool ok_run = (n1 == 19) && parse_motor_stream_response(rx, n1);
      if (ok_run) {
        ext.position_um   = tele.position_um;
        ext.force_mn      = tele.force_mn;
        ext.power_w       = tele.power_w;
        ext.temperature_c = tele.temperature_c;
        ext.voltage_mv    = tele.voltage_mv;
        ext.errors        = tele.errors;
      }

      // ---- Tx2: 0x03 read shaft speed (2 regs, 32-bit) ----
      elapsedMicros t2;
      tx_len = build_read_holding(tx, REG_SHAFT_SPEED, 2);
      int n2 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 9);
      ext.t_speed_us = (uint32_t)t2;
      bool ok_speed = parse_read_holding_2reg_int32(rx, n2, &ext.shaft_speed_mmps);

      // ---- Tx3: 0x03 read shaft acceleration (2 regs, 32-bit) ----
      elapsedMicros t3;
      tx_len = build_read_holding(tx, REG_SHAFT_ACCEL, 2);
      int n3 = modbus_transact(tx, tx_len, rx, sizeof(rx), 5000, 9);
      ext.t_accel_us = (uint32_t)t3;
      bool ok_accel = parse_read_holding_2reg_int32(rx, n3, &ext.shaft_accel_mmpss);

      ext.t_total_us = (uint32_t)cycle_t;
      ext.seq++;
      ext.valid = ok_run && ok_speed && ok_accel;

      if (ext.valid) {
        // Push immediately for full-rate timing data on the Mac side.
        send_ext_telemetry();

        // Error-register rising edge
        static uint16_t last_ext_errors = 0;
        if (ext.errors != 0 && ext.errors != last_ext_errors) {
          char msg[64];
          snprintf(msg, sizeof(msg), "MOTOR_ERR_REG=0x%04X", ext.errors);
          send_error(msg);
        }
        last_ext_errors = ext.errors;
      } else {
        static elapsedMillis since_last_warn;
        if (since_last_warn > 1000) {
          char msg[96];
          snprintf(msg, sizeof(msg),
                   "EXT_CYCLE_BAD run=%d speed=%d accel=%d (lens %d %d %d)",
                   ok_run, ok_speed, ok_accel, n1, n2, n3);
          send_error(msg);
          since_last_warn = 0;
        }
      }
    } else {
      // === BASIC MODE: single 0x64 per cycle (existing behavior) ===
      size_t tx_len = build_motor_stream(tx, stream_sub, stream_data);
      int n = modbus_transact(tx, tx_len, rx, sizeof(rx),
                              stream_period_us > 5000 ? stream_period_us : 5000,
                              19);

      if (n == 19 && parse_motor_stream_response(rx, n)) {
        static uint16_t last_errors = 0;
        if (tele.errors != 0 && tele.errors != last_errors) {
          char msg[64];
          snprintf(msg, sizeof(msg), "MOTOR_ERR_REG=0x%04X", tele.errors);
          send_error(msg);
        }
        last_errors = tele.errors;
      } else {
        static elapsedMillis since_last_warn;
        if (since_last_warn > 1000) {
          send_error("STREAM_RESPONSE_BAD");
          since_last_warn = 0;
        }
      }
    }
  }

  // ---- Send telemetry on schedule (independent of stream rate) ----
  // In extended mode, telemetry is pushed every cycle by send_ext_telemetry,
  // so we skip the periodic send here.
  if (!extended_mode && tele.valid && since_last_telemetry >= telemetry_period_us) {
    since_last_telemetry = 0;
    send_telemetry();
  }
}
