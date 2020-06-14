// DISCLAIMER: This is not intended to be used for any real-world safety applications - it is for educational/informational purposes only! Use at your own risk.
//
// board=MiniCore:avr:328:bootloader=uart0,variant=modelP,BOD=2v7,LTO=Os_flto,clock=8MHz_external
// port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
// verbose=1

#include <avr/wdt.h>
#include <avr/power.h>
#include <ArduinoJson.h>
#include <SPI.h>

#define CS PIN_PD4
#define MOSI PIN_PB3
#define MISO PIN_PB4
#define SCLK PIN_PB5
#define GDO0 PIN_PD3
#define GDO2 PIN_PC0
#define EN PIN_PC5
#define LED PIN_PB0

#define ORIG_SCLK PIN_PC2
#define ORIG_MOSI PIN_PC3
#define ORIG_MISO PIN_PC1

SPISettings CC1101(1000000, MSBFIRST, SPI_MODE0);

// DIP switches on alarm
#define ADDRESS 0b00000000

// number of seconds without receiving another packet after which the alarm is considered inactive
#define ALARM_DURATION 5

// LOG_LEVEL can be one of ERROR, INFO, DEBUG, or TRACE
// running with TRACE or even DEBUG will likely cause packet loss - not recommended outside of testing
#define LOG_LEVEL INFO

// PROMISC_MODE will output packets to the uart even if they don't match ADDRESS
// note that the alarm and ALARM_DURATION functionality is disabled when this is set to 1
// JSON objects will instead be type "packet"
#define PROMISC_MODE 0

// some commands appear to work without the MSB (0x80) set. however, the alarm itself always seems to set it
#define KIDDE_COMMAND (0x80)
#define KIDDE_TEST (KIDDE_COMMAND | 0x00)
#define KIDDE_HUSH (KIDDE_COMMAND | 0x01)
#define KIDDE_CO (KIDDE_COMMAND | 0x02)
#define KIDDE_SMOKE (KIDDE_COMMAND | 0x03)
#define KIDDE_BATTERY (KIDDE_COMMAND | 0x0D)

#define CC1101_STROBE_SRES 0x30
#define CC1101_STROBE_SCAL 0x33
#define CC1101_STROBE_SRX 0x34
#define CC1101_STROBE_STX 0x35
#define CC1101_STROBE_SIDLE 0x36
#define CC1101_STROBE_SFRX 0x3A
#define CC1101_STROBE_SFTX 0x3B
#define CC1101_STROBE_SNOP 0x3D

#define CC1101_STATUS_CAL 0x40

#define CC1101_CMD_WRITE 0x00
#define CC1101_CMD_WRITE_BURST 0x40
#define CC1101_CMD_READ 0x80
#define CC1101_CMD_READ_BURST 0xC0

#define CC1101_REG_IOCFG2 0x00
#define CC1101_REG_PARTNUM 0x30
#define CC1101_REG_VERSION 0x31
#define CC1101_REG_PKTSTATUS 0x38
#define CC1101_REG_RXBYTES 0x3B

#define CC1101_RX_FIFO 0x3F
#define CC1101_TX_FIFO 0x3F

#define CC1101_RX_OVERFLOW 0x80

#define ERROR 0
#define INFO 1
#define DEBUG 2
#define TRACE 3

#define LOG(_LEVEL_, _MSG_, ...) \
  if (LOG_LEVEL >= _LEVEL_) { \
    char buffer[128]; \
    snprintf_P(buffer, 128, (PGM_P) F(_MSG_), ##__VA_ARGS__); \
    logSerial(_LEVEL_, buffer, __func__); \
  }

#define _start LOG(TRACE, "start")
#define _end LOG(TRACE, "end")

// obtained via pulseview + cc1101 decoder on wink hub 1
// Burst write: IOCFG2 (00) = 07 2E 04 07 D3 91 04 0C 44 00 00 06 00 10 AA 80 4B F9 03 22 F7 46 07 0F 18 1D 1C C7 00 B2 30 AA 69 B6 10 EA 2A 00
static const uint8_t kidde_regs[] PROGMEM = {
  // 0x00 ...
  0x07, // IOCFG2   GDO2 Output Pin Configuration
  0x2e, // IOCFG1   GDO1 Output Pin Configuration
  0x04, // IOCFG0   GDO0 Output Pin Configuration
  0x07, // FIFOTHR  RX FIFO and TX FIFO Thresholds
  0xD3, // SYNC1    Sync word, high byte
  0x91, // SYNC0    Sync word, low byte
  0x04, // PKTLEN   Packet Length
  0x0C, // PKTCTRL1 Packet Automation Control
  0x44, // PKTCTRL0 Packet Automation Control
  0x00, // ADDR     Device Address
  0x00, // CHANR    Channel Number
  0x06, // FSCTRL1  Frequency Synthesizer Control
  0x00, // FSCTRL0  Frequency Synthesizer Control
  0x10, // FREQ2    Frequency Control Word, High Byte
  0xAA, // FREQ1    Frequency Control Word, Middle Byte
  0x80, // FREQ0    Frequency Control Word, Low Byte
  0x4B, // MDMCFG4  Modem Configuration
  0xF9, // MDMCFG3  Modem Configuration
  0x03, // MDMCFG2  Modem Configuration
  0x22, // MDMCFG1  Modem Configuration
  0xF7, // MDMCFG0  Modem Configuration
  0x46, // DEVIATN  Modem Deviation Setting
  0x07, // MCSM2    Main Radio Control State Machine Configuration
  0x0F, // MCSM1    Main Radio Control State Machine Configuration
  0x18, // MCSM0    Main Radio Control State Machine Configuration
  0x1D, // FOCCFG   Frequency Offset Compensation Configuration
  0x1C, // BSCFG    Bit Synchronization Configuration
  0xC7, // AGCCTRL2 AGC Control
  0x00, // AGCCTRL1 AGC Control
  0xB2, // AGCCTRL0 AGC Control
  0x30, // WOREVT1  High Byte Event0 Timeout
  0xAA, // WOREVT0  Low Byte Event0 Timeout
  0x69, // WORCTRL  Wake On Radio Control
  0xB6, // FREND1   Front End RX Configuration
  0x10, // FREND0   Front End TX Configuration
  0xEA, // FSCAL3   Frequency Synthesizer Calibration
  0x2A, // FSCAL2   Frequency Synthesizer Calibration
  0x00  // FSCAL1   Frequency Synthesizer Calibration
  // ... 0x25
};

struct kidde_pkt {
  uint8_t address;
  uint8_t command;
  uint8_t suffix[2];
  uint8_t rssi;
  uint8_t crc_lqi;
};

struct alarm_state {
  bool active;
  uint32_t count;
  unsigned long first;
  unsigned long last;
  uint8_t min_rssi;
  uint8_t max_rssi;
};

enum kidde_cmd {
  TEST,
  HUSH,
  CO,
  SMOKE,
  BATTERY,
  UNKNOWN,
  NUM_CMDS
};

struct alarm_state alarm_states[NUM_CMDS];

static const char *kidde_strs[] = {
  "test",
  "hush",
  "co",
  "smoke",
  "battery",
  "unknown"
};

enum kidde_cmd kiddeCmdMap(uint8_t command) {
  _start;

  enum kidde_cmd mapped;

  switch (command) {
    case KIDDE_TEST:
      mapped = TEST;
      break;
    case KIDDE_HUSH:
      mapped = HUSH;
      break;
    case KIDDE_CO:
      mapped = CO;
      break;
    case KIDDE_SMOKE:
      mapped = SMOKE;
      break;
    case KIDDE_BATTERY:
      mapped = BATTERY;
      break;
    default:
      mapped = UNKNOWN;
      break;
  }

  _end;

  return (mapped);
}

// send a log message over uart
void logSerial(uint8_t level, const char *msg, const char *caller) {
  StaticJsonDocument<128> log;

  log["millis"] = millis();
  log["type"] = "log";
  log["msg"] = msg;
  log["caller"] = caller;

  switch (level) {
    case TRACE:
      log["level"] = "trace";
      break;
    case DEBUG:
      log["level"] = "debug";
      break;
    case INFO:
      log["level"] = "info";
      break;
    case ERROR:
      log["level"] = "error";
      break;
    default:
      log["level"] = level;
      break;
  }

  serializeJson(log, Serial);
  Serial.println();
}

void enableWatchdog() {
  _start;

  wdt_enable(WDTO_8S);

  _end;
}

void updateWatchdog() {
  _start;

  wdt_reset();

  _end;
}

void resetBoard() {
  _start;

  void (*reset) (void) = 0;

  LOG(INFO, "resetting board");

  _end;

  reset();
}

void setupPins() {
  _start;

  // disable original bitbanged spi pins
  pinMode(ORIG_SCLK, INPUT);
  pinMode(ORIG_MOSI, INPUT);
  pinMode(ORIG_MISO, INPUT);

  // set CS as output
  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);

  // set MISO as input
  pinMode(MISO, INPUT);

  // Asserts when the RX FIFO has overflowed. De-asserts when the FIFO has been flushed.
  pinMode(GDO0, INPUT);
  digitalWrite(GDO0, LOW);

  // Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO.
  pinMode(GDO2, INPUT);
  digitalWrite(GDO2, LOW);

  // enable amp on RM1101-USB-232, active high
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);

  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  _end;
}

void ledBlink() {
  _start;

  // blink a few times to signify reset
  for (int x = 0; x < 5; x++) {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }

  _end;
}

void disableUnusedHw() {
  _start;

  power_adc_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();

  _end;
}

void spiInit() {
  _start;

  SPI.begin();

  _end;
}

void spiBegin() {
  SPI.beginTransaction(CC1101);

  digitalWrite(CS, LOW);
}

void spiEnd() {
  digitalWrite(CS, HIGH);

  SPI.endTransaction();
}

uint8_t readReg(uint8_t reg) {
  _start;

  uint8_t val;

  spiBegin();

  SPI.transfer(reg);
  val = SPI.transfer(reg);

  spiEnd();

  _end;

  return (val);
}

void resetCC1101() {
  _start;

  unsigned long start_time = micros();

  // reset sequence per datasheet
  pinMode(MISO, INPUT);
  digitalWrite(MISO, HIGH);
  digitalWrite(CS, LOW);
  delay(1);
  digitalWrite(CS, HIGH);
  delay(1);
  digitalWrite(CS, LOW);
  while (digitalRead(MISO) == HIGH);

  spiBegin();

  SPI.transfer(CC1101_STROBE_SRES);
  while (digitalRead(MISO) == HIGH);

  spiEnd();

  LOG(INFO, "reset took %lu us", micros() - start_time);

  _end;
}

void chipInfo() {
  _start;

  StaticJsonDocument<128> chip_info;

  chip_info["type"] = "chip_info";

  // get status
  chip_info["status"] = readReg(CC1101_STROBE_SNOP);

  // get partnum
  chip_info["partnum"] = readReg(CC1101_CMD_READ_BURST | CC1101_REG_PARTNUM);

  // get version
  chip_info["version"] = readReg(CC1101_CMD_READ_BURST | CC1101_REG_VERSION);

  serializeJson(chip_info, Serial);
  Serial.println();

  _end;
}

void writeKiddeRegs() {
  _start;

  uint8_t kidde_regs_m[sizeof(kidde_regs)];
  // save a few bytes of ram
  memcpy_P(kidde_regs_m, kidde_regs, sizeof(kidde_regs));

  // burst write regs for kidde receiver
  spiBegin();

  SPI.transfer(CC1101_CMD_WRITE_BURST | CC1101_REG_IOCFG2);
  for (uint8_t reg = 0; reg < sizeof(kidde_regs_m); reg++) {
    SPI.transfer(kidde_regs_m[reg]);
  }

  spiEnd();

  _end;
}

void verifyKiddeRegs() {
  _start;

  uint8_t kidde_regs_m[sizeof(kidde_regs)];
  bool error = false;
  // save a few bytes of ram
  memcpy_P(kidde_regs_m, kidde_regs, sizeof(kidde_regs));

  // burst read regs back to verify
  spiBegin();

  SPI.transfer(CC1101_CMD_READ_BURST | CC1101_REG_IOCFG2);
  for (uint8_t reg = 0; reg < sizeof(kidde_regs_m); reg++) {
    uint8_t val = SPI.transfer(0);
    if (val != kidde_regs_m[reg]) {
      error = true;

      LOG(ERROR, "mismatch on read of 0x%02hhx: expected 0x%02hhx, got 0x%02hhx", reg, kidde_regs_m[reg], val);
    }
  }

  spiEnd();

  if (error)
    resetBoard();

  _end;
}

void calibrateCC1101() {
  _start;

  unsigned long start_time = micros();

  // frequency synthesizer calibration
  spiBegin();

  SPI.transfer(CC1101_STROBE_SCAL);
  while (SPI.transfer(CC1101_STROBE_SNOP) & CC1101_STATUS_CAL)
    delay(1);

  spiEnd();

  LOG(INFO, "calibration took %lu us", micros() - start_time);

  _end;
}

void startRx() {
  _start;

  // put radio into receive mode
  spiBegin();

  SPI.transfer(CC1101_STROBE_SRX);

  spiEnd();

  _end;
}

void flushRxFifo() {
  _start;

  // flush rx fifo
  spiBegin();

  SPI.transfer(CC1101_STROBE_SFRX);

  spiEnd();

  _end;
}

void discardFifoBytes(uint8_t count) {
  _start;

  spiBegin();

  SPI.transfer(CC1101_CMD_READ_BURST | CC1101_RX_FIFO);

  for (int pos = 0; pos < count; pos++) {
    SPI.transfer(0);
  }

  spiEnd();

  _end;
}

void rxPacket(struct kidde_pkt *pkt) {
  _start;

  memset(pkt, 0, sizeof(kidde_pkt));

  // pull a packet's worth of bytes from the fifo
  spiBegin();

  SPI.transfer(CC1101_CMD_READ_BURST | CC1101_RX_FIFO);

  pkt->address = SPI.transfer(0);
  pkt->command = SPI.transfer(0);
  pkt->suffix[0] = SPI.transfer(0);
  pkt->suffix[1] = SPI.transfer(0);
  pkt->rssi = SPI.transfer(0);
  pkt->crc_lqi = SPI.transfer(0);

  spiEnd();

  _end;
}

#if PROMISC_MODE
void printPacket(struct kidde_pkt *pkt) {
  _start;

  StaticJsonDocument<128> msg;

  msg["millis"] = millis();
  msg["type"] = "packet";
  msg["command"] = kidde_strs[kiddeCmdMap(pkt->command)];
  msg["address"] = pkt->address;
  msg["raw_command"] = pkt->command;
  msg["suffix"] = (uint16_t) (pkt->suffix[0] << 8) + pkt->suffix[1];
  msg["rssi"] = pkt->rssi;
  msg["crc"] = (bool) (pkt->crc_lqi >> 7);
  msg["lqi"] = (pkt->crc_lqi & ~0x80);

  serializeJson(msg, Serial);
  Serial.println();

  _end;
}
#else

bool updateAlarmState(struct kidde_pkt *pkt, struct alarm_state *state) {
  _start;

  bool is_new = false;

  if (!state->active) {
    state->first = millis();
    is_new = state->active = true;
    state->min_rssi = state->max_rssi = pkt->rssi;
  }
  else {
    if (pkt->rssi < state->min_rssi)
      state->min_rssi = pkt->rssi;

    if (pkt->rssi > state->max_rssi)
      state->max_rssi = pkt->rssi;
  }

  state->count++;
  state->last = millis();

  _end;

  return (is_new);
}

void printAlarmState(uint8_t command, struct alarm_state *state) {
  _start;

  StaticJsonDocument<128> msg;

  msg["millis"] = millis();
  msg["type"] = "alarm";
  msg["command"] = kidde_strs[command];
  msg["min_rssi"] = state->min_rssi;
  msg["max_rssi"] = state->max_rssi;
  msg["duration"] = state->last - state->first;
  msg["count"] = state->count;
  msg["active"] = state->active;

  serializeJson(msg, Serial);
  Serial.println();

  _end;
}

void expireAlarm(uint8_t command, struct alarm_state *state) {
  _start;

  state->active = false;

  printAlarmState(command, state);

  memset(state, 0, sizeof(alarm_state));

  _end;
}

void initAlarmStates() {
  _start;

  struct alarm_state *state;

  for (int command = 0; command != NUM_CMDS; ++command) {
    state = &alarm_states[command];

    memset(&state, 0, sizeof(alarm_state));
  }

  _end;
}

void expireAlarmStates() {
  _start;

  struct alarm_state *state;

  for (int command = 0; command != NUM_CMDS; ++command) {
    state = &alarm_states[command];

    if (!state->active)
      continue;

    if ((millis() - state->last) < (ALARM_DURATION * 1000))
      continue;

    expireAlarm(command, state);
  }

  _end;
}
#endif

void processRxFifo() {
  _start;

  uint8_t rx_bytes;
  struct kidde_pkt pkt;
  enum kidde_cmd mapped_cmd;
  struct alarm_state *state;
  uint8_t max_packets = 32; // handle the rest on the next loop() iteration

  // blinky
  digitalWrite(LED, HIGH);

  // fetch bytes from fifo and parse packets
  while (max_packets && (rx_bytes = readReg(CC1101_CMD_READ_BURST | CC1101_REG_RXBYTES)) >= sizeof(kidde_pkt)) {
    if (rx_bytes & CC1101_RX_OVERFLOW) {
      LOG(ERROR, "rx fifo overflow. flushing");

      discardFifoBytes(rx_bytes & ~CC1101_RX_OVERFLOW);

      flushRxFifo();

      break;
    }

    LOG(DEBUG, "got packet. rx fifo has %u bytes", rx_bytes);

    rxPacket(&pkt);

    // don't let a flood of packets starve mcu (and watchdog)
    max_packets--;

#if PROMISC_MODE
    // print all packets, even if they're not addressed to us
    // alarm state is not kept due to memory constraints
    printPacket(&pkt);
#else
    // don't care
    if (pkt.address != ADDRESS)
      continue;

    mapped_cmd = kiddeCmdMap(pkt.command);
    state = &alarm_states[mapped_cmd];

    // if there's already an active, unexpired alarm, take note and move on
    if (!updateAlarmState(&pkt, state))
      continue;

    printAlarmState(mapped_cmd, state);
#endif
  }

  digitalWrite(LED, LOW);

  _end;
}

void setup() {
  // so usb uart can enumerate
  if (LOG_LEVEL == TRACE)
    delay(1000);

  Serial.begin(115200);

  _start;

  LOG(INFO, "setup");

  enableWatchdog();

  setupPins();

  disableUnusedHw();

  ledBlink();

  spiInit();

  resetCC1101();

  chipInfo();

  writeKiddeRegs();

  verifyKiddeRegs();

  calibrateCC1101();

#if !PROMISC_MODE
  initAlarmStates();
#endif

  startRx();

  LOG(DEBUG, "entering loop");

  _end;
}

void loop() {
  uint16_t max_delay = 1000; // ~1s

  // set to 8s
  updateWatchdog();

  // wait until GDO2 goes HIGH - this signifies a packet has been received
  while (digitalRead(GDO2) == LOW && max_delay) {
    delay(1);
    max_delay--;
  }

  if (digitalRead(GDO2) == HIGH)
    processRxFifo();

#if !PROMISC_MODE
  expireAlarmStates();
#endif
}
