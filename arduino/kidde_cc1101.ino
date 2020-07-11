// DISCLAIMER: This is not intended to be used for any real-world safety applications - it is for educational/informational purposes only! Use at your own risk.
//
// board=MiniCore:avr:328:bootloader=uart0,variant=modelP,BOD=2v7,LTO=Os_flto,clock=8MHz_external
// port=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
// verbose=1

#include <avr/wdt.h>
#include <avr/power.h>
#include <util/crc16.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <EEPROM.h>
#include <stdarg.h>

// LOG_LEVEL can be one of ERROR, INFO, DEBUG, or TRACE
// running with TRACE or even DEBUG will likely cause packet loss - not recommended outside of testing
#define LOG_LEVEL INFO

// uncomment this to allow adjusting log level during runtime, otherwise it will be fixed at LOG_LEVEL
// note this will increase flash usage by ~6KiB vs compiling only with INFO or ERROR
#define DYNAMIC_LOG

// uncomment this to gather memory statistics (for TRACE)
// requires library from https://github.com/McNeight/MemoryFree
#define MEMORY_USAGE

// max number of addresses to monitor. limited by ram - each monitored address requires ~96 bytes of ram.
#define ADDRESS_MAX 4

// read this many bytes from uart at a go. effectively limits max json command length.
#define SERIAL_BUFFER 64

// milliseconds to wait for an "\n" upon receiving a byte from the uart. 9600 baud would require < 1ms.
#define SERIAL_TIMEOUT 100

enum log_level {
  ERROR,
  INFO,
  DEBUG,
  TRACE,
  NUM_LEVELS
};

static const char *log_strs[] = {
  "error",
  "info",
  "debug",
  "trace"
};

struct settings {
  uint8_t address[ADDRESS_MAX];
  uint8_t address_cnt;
  uint16_t expiry;
  uint32_t baud;
  bool promisc;
#ifdef DYNAMIC_LOG
  uint8_t log_level;
#endif
} s = {               // defaults below if eeprom is blank/corrupt - adjustable during runtime and saved in eeprom
  {0},                // default address of 0x00
  1,                  // only 1 address
  10,                 // 10 second expiry
  38400,              // works well for 8MHz and 16MHz xtals. 500000 recommended on 8MHz.
  0,                  // unicast only
#ifdef DYNAMIC_LOG
  LOG_LEVEL           // default loglevel
#endif
};

uint8_t s_max_packets; // derived based on baud

// NOTE: pins below will likely need to be changed unless you're using a atmega328p swapped onto a RM1101-USB-232 like me :)

// defaults for RM1101-USB-232
//#define RM1101_USB_232

// for standard arduino ide core
#ifdef ARDUINO_AVR_UNO

#define CS PD4
#define MOSI PB3
#define MISO PB4
#define SCLK PB5
#define GDO0 PD3
#define GDO2 PC0
#define LED PB0

#ifdef RM1101_USB_232
#define EN PC5
#define ORIG_SCLK PC2
#define ORIG_MOSI PC3
#define ORIG_MISO PC1
#endif

#endif

// for "minicore" arduino core
#ifdef ARDUINO_AVR_ATmega328

#define CS PIN_PD4
#define MOSI PIN_PB3
#define MISO PIN_PB4
#define SCLK PIN_PB5
#define GDO0 PIN_PD3
#define GDO2 PIN_PC0
#define LED PIN_PB0

#ifdef RM1101_USB_232
#define EN PIN_PC5
#define ORIG_SCLK PIN_PC2
#define ORIG_MOSI PIN_PC3
#define ORIG_MISO PIN_PC1
#endif

#endif

SPISettings CC1101(1000000, MSBFIRST, SPI_MODE0);

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

// call stack depth, for TRACE
uint8_t depth = 0;

#define LOG_BUFFER_SIZE 128

#ifdef DYNAMIC_LOG
#define _LOG_LEVEL_ s.log_level
#else
#define _LOG_LEVEL_ LOG_LEVEL
#endif

#define LOG(_LEVEL_, _MSG_, ...) \
  if (_LOG_LEVEL_ >= _LEVEL_) \
    logSerial(_LEVEL_, _func_name_, PSTR(_MSG_), ##__VA_ARGS__);

// using __func__ here eats a ton of ram. hence manually passing/setting _FUNC_NAME_ once per function.
#ifdef MEMORY_USAGE
#include <MemoryFree.h>
#define _start(_FUNC_NAME_) \
  depth++; \
  const static char _func_name_[] PROGMEM = _FUNC_NAME_; \
  LOG(TRACE, "start: %i", freeMemory());
#define _end \
  LOG(TRACE, "end: %i", freeMemory()) \
  depth--;
#else
#define _start(_FUNC_NAME_) \
  depth++; \
  const static char _func_name_[] PROGMEM = _FUNC_NAME_; \
  LOG(TRACE, "start");
#define _end \
  LOG(TRACE, "end") \
  depth--;
#endif

static const char EEPROM_MAGIC[] = {'k','i','d','d','e'};

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

static const char *kidde_strs[] = {
  "test",
  "hush",
  "co",
  "smoke",
  "battery",
  "unknown"
};

struct alarm_state alarm_states[ADDRESS_MAX][NUM_CMDS];

void logSerial(uint8_t level, const char *caller PROGMEM, const char *msg PROGMEM, ...) {
  // 32 = max 'caller' length. 'msg' isn't duplicated as it's cast to const char.
  StaticJsonDocument<JSON_OBJECT_SIZE(6) + 32> log;
  char buffer[LOG_BUFFER_SIZE];
  va_list args;

  va_start(args, msg);
  vsnprintf_P(buffer, LOG_BUFFER_SIZE, msg, args);
  va_end(args);

  log["millis"] = millis();
  log["type"] = "log";
  log["level"] = log_strs[level];
  log["caller"] = (const __FlashStringHelper *) caller;
  if (_LOG_LEVEL_ == TRACE)
    log["depth"] = depth;
  log["msg"] = (const char *) buffer;

  serializeJson(log, Serial);
  Serial.println();
}

enum kidde_cmd kiddeCmdMap(uint8_t command) {
  _start("kiddeCmdMap");

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

void enableWatchdog() {
  _start("enableWatchdog");

  wdt_enable(WDTO_8S);

  _end;
}

void updateWatchdog() {
  _start("updateWatchdog");

  wdt_reset();

  _end;
}

void resetBoard() {
  _start("resetBoard");

  LOG(INFO, "resetting board");

  _end;

  wdt_enable(WDTO_1S);
  while (1);
}

void setupPins() {
  _start("setupPins");

#ifdef RM1101_USB_232
  // disable original bitbanged spi pins on RM1101-USB-232
  pinMode(ORIG_SCLK, INPUT);
  pinMode(ORIG_MOSI, INPUT);
  pinMode(ORIG_MISO, INPUT);
#endif

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

#ifdef RM1101_USB_232
  // enable amp on RM1101-USB-232, active high
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);
#endif

  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  _end;
}

void ledBlink() {
  _start("ledBlink");

  // blink a few times to signify reset
  for (uint8_t x = 0; x < 5; x++) {
    digitalWrite(LED, HIGH);
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }

  _end;
}

void disableUnusedHw() {
  _start("disableUnusedHw");

  power_adc_disable();
  power_timer1_disable();
  power_timer2_disable();
  power_twi_disable();

  _end;
}

void initSerial() {
  // clear tx fifo / rx buffer
  Serial.end();

  // per-byte timeout of incoming commands
  Serial.setTimeout(SERIAL_TIMEOUT);

  Serial.begin(s.baud);

  // adjust rx loop based on baud
  setMaxPackets();
}

void setMaxPackets() {
  if (s.baud > 115200) {
    s_max_packets = 32;
  }
  else if (s.baud > 57600) {
    s_max_packets = 16;
  }
  else if (s.baud > 28800) {
    s_max_packets = 8;
  }
  else if (s.baud > 14400) {
    s_max_packets = 4;
  }
  else {
    s_max_packets = 2;
  }
}

void spiInit() {
  _start("spiInit");

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
  _start("readReg");

  uint8_t val;

  spiBegin();

  SPI.transfer(reg);
  val = SPI.transfer(reg);

  spiEnd();

  _end;

  return (val);
}

void resetCC1101() {
  _start("resetCC1101");

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
  _start("chipInfo");

  StaticJsonDocument<JSON_OBJECT_SIZE(4)> chip_info;

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
  _start("writeKiddeRegs");

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
  _start("verifyKiddeRegs");

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
  _start("calibrateCC1101");

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
  _start("startRx");

  // put radio into receive mode
  spiBegin();

  SPI.transfer(CC1101_STROBE_SRX);

  spiEnd();

  _end;
}

void flushRxFifo() {
  _start("flushRxFifo");

  // flush rx fifo
  spiBegin();

  SPI.transfer(CC1101_STROBE_SFRX);

  spiEnd();

  _end;
}

void discardFifoBytes(uint8_t count) {
  _start("discardFifoBytes");

  spiBegin();

  SPI.transfer(CC1101_CMD_READ_BURST | CC1101_RX_FIFO);

  for (int pos = 0; pos < count; pos++) {
    SPI.transfer(0);
  }

  spiEnd();

  _end;
}

void rxPacket(struct kidde_pkt *pkt) {
  _start("rxPacket");

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

void printPacket(struct kidde_pkt *pkt) {
  _start("printPacket");

  StaticJsonDocument<JSON_OBJECT_SIZE(9)> msg;

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

void initAlarmStates() {
  _start("initAlarmStates");

  memset(alarm_states, 0, sizeof(alarm_state) * NUM_CMDS * ADDRESS_MAX);

  _end;
}

bool updateAlarmState(struct kidde_pkt *pkt, struct alarm_state *state) {
  _start("updateAlarmState");

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

void printAlarmState(uint8_t address, uint8_t command, struct alarm_state *state) {
  _start("printAlarmState");

  StaticJsonDocument<JSON_OBJECT_SIZE(9)> msg;

  msg["millis"] = millis();
  msg["type"] = "alarm";
  msg["address"] = address;
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

void expireAlarm(uint8_t address, uint8_t command, struct alarm_state *state) {
  _start("expireAlarm");

  state->active = false;

  printAlarmState(address, command, state);

  memset(state, 0, sizeof(alarm_state));

  _end;
}

void expireAlarmStates() {
  _start("expireAlarmStates");

  struct alarm_state *state;

  for (uint8_t address = 0; address < s.address_cnt; address++) {
    for (uint8_t command = 0; command < NUM_CMDS; command++) {
      state = &alarm_states[address][command];

      if (!state->active)
        continue;

      if ((millis() - state->last) < (s.expiry * 1000))
        continue;

      expireAlarm(s.address[address], command, state);
    }
  }

  _end;
}

void expireAllAlarms() {
  _start("expireAllAlarms");

  uint16_t expiry = s.expiry;
  // force any active alarms to expire
  s.expiry = 0;
  expireAlarmStates();
  s.expiry = expiry;
  initAlarmStates();

  _end;
}

void processRxFifo() {
  _start("processRxFifo");

  uint8_t rx_bytes;
  struct kidde_pkt pkt;
  enum kidde_cmd mapped_cmd;
  bool matched;
  uint8_t address_idx;
  struct alarm_state *state;
  // handle the rest on the next loop() iteration
  uint8_t max_packets = s_max_packets;

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

    // print all packets, even if they're not addressed to us
    // alarm state is not kept due to memory constraints
    if (s.promisc) {
      printPacket(&pkt);
      continue;
    }

    // see if it's addressed to one of the addresses we care about
    for (matched = false, address_idx = 0; address_idx < s.address_cnt; address_idx++) {
      if (pkt.address == s.address[address_idx]) {
        matched = true;
        break;
      }
    }

    // don't care
    if (!matched)
      continue;

    mapped_cmd = kiddeCmdMap(pkt.command);
    state = &alarm_states[address_idx][mapped_cmd];

    // if there's already an active, unexpired alarm, take note and move on
    if (!updateAlarmState(&pkt, state))
      continue;

    printAlarmState(pkt.address, mapped_cmd, state);
  }

  digitalWrite(LED, LOW);

  _end;
}

bool eepromReadConfig() {
  //_start("eepromReadConfig");

  bool valid = false;
  char magic[sizeof(EEPROM_MAGIC)];
  uint16_t crc = 0;
  struct settings S;

  EEPROM.get(0, magic);

  if (!strncmp(magic, EEPROM_MAGIC, sizeof(EEPROM_MAGIC))) {
    EEPROM.get(sizeof(magic), crc);

    //LOG(DEBUG, "crc is 0x%X", crc);

    EEPROM.get(sizeof(EEPROM_MAGIC) + sizeof(crc), S);

    if (crc == calcChecksum(&S)) {
      s = S;
      valid = true;
    }
    //else {
    //  LOG(ERROR, "checksum mismatch. calculated 0x%X != actual 0x%X", calcChecksum(&S), crc);
    //}
  }
  //else {
  //  LOG(ERROR, "bad magic: %s", magic);
  //}

  //_end;

  return (valid);
}

void eepromWriteConfig() {
  _start("eepromWriteConfig");

  uint8_t pos;
  uint16_t crc = calcChecksum(&s);
  uint8_t *s_bytes = (uint8_t *) &s;

  for (pos = 0; pos < sizeof(EEPROM_MAGIC); pos++) {
    LOG(DEBUG, "magic: wrote 0x%02X to %u", EEPROM_MAGIC[pos], pos);
    EEPROM.put(pos, EEPROM_MAGIC[pos]);
  }

  EEPROM.put(pos, (byte) (crc & 0xFF));
  LOG(DEBUG, "crc: wrote 0x%02X to %u", (byte) (crc & 0xFF), pos);
  EEPROM.put(++pos, (byte) (crc >> 8));
  LOG(DEBUG, "crc: wrote 0x%02X to %u", (byte) (crc >> 8), pos);

  for (uint8_t b = 0; b < sizeof(s); b++) {
    EEPROM.put(++pos, s_bytes[b]);
    LOG(DEBUG, "s: wrote 0x%02X to %u", s_bytes[b], pos);
  }

  LOG(INFO, "wrote settings to eeprom");

  _end;
}

void eepromClear() {
  _start("eepromClear");

  uint8_t length = sizeof(EEPROM_MAGIC) + sizeof(uint16_t) + sizeof(settings);
  for (uint8_t pos = 0; pos < length; pos++)
    EEPROM.put(pos, 0xFF);

  LOG(INFO, "eeprom cleared");

  _end;
}

uint16_t calcChecksum(struct settings *S) {

  uint8_t *bytes = (uint8_t *) S;
  uint16_t crc = 0;

  for (uint8_t num = 0; num < sizeof(settings); num++)
    crc = _crc16_update(crc, bytes[num]);

  //LOG(DEBUG, "checksum is 0x%02X", crc);

  return (crc);
}

void dumpConfig() {
  _start("dumpConfig");

  StaticJsonDocument<JSON_OBJECT_SIZE(4) + JSON_ARRAY_SIZE(ADDRESS_MAX)> config;

  JsonArray address = config.createNestedArray("address");
  for (uint8_t num = 0; num < s.address_cnt; num++)
    address.add(s.address[num]);

  config["expiry"] = s.expiry;
  config["baud"] = s.baud;
  config["promisc"] = s.promisc;

  serializeJson(config, Serial);
  Serial.println();

  _end;
}

void parseCommand(char *input) {
  _start("parseCommand");

  StaticJsonDocument<JSON_OBJECT_SIZE(3) + JSON_ARRAY_SIZE(ADDRESS_MAX)> command;
  DeserializationError err = deserializeJson(command, input);

  if (err) {
    LOG(ERROR, "bad json: %s", input);
  }
  else if (command["type"] == "get") {
    getConfig(command);
  }
  else if (command["type"] == "set") {
    setConfig(command);
  }
  else {
    LOG(ERROR, "'type' unknown");
  }

  _end;
}

void getConfig(const JsonDocument& command) {
  _start("getConfig");

  StaticJsonDocument<JSON_OBJECT_SIZE(3) + JSON_ARRAY_SIZE(ADDRESS_MAX)> response;
  bool success = true;

  response["type"] = "config";

  if (command["key"] == "address") {
    JsonArray address = response.createNestedArray("value");
    for (uint8_t num = 0; num < s.address_cnt; num++)
      address.add(s.address[num]);
  }
  else if (command["key"] == "expiry") {
    response["value"] = s.expiry;
  }
  else if (command["key"] == "baud") {
    response["value"] = s.baud;
  }
  else if (command["key"] == "promisc") {
    response["value"] = s.promisc;
  }
#ifdef DYNAMIC_LOG
  else if (command["key"] == "log_level") {
    response["value"] = log_strs[s.log_level];
  }
#endif
  else {
    LOG(ERROR, "'key' unknown");
    success = false;
  }

  if (success) {
    response["key"] = command["key"];
    serializeJson(response, Serial);
    Serial.println();
  }

  _end;
}

void setConfig(const JsonDocument& command) {
  _start("setConfig");

  bool success = false;

  if (command["key"] == "address") {
    JsonArrayConst json_address = command["value"].as<JsonArrayConst>();
    success = setAddress(json_address);
  }
  else if (command["key"] == "baud") {
    JsonVariantConst json_baud = command["value"].as<JsonVariantConst>();
    success = setBaud(json_baud);
  }
  else if (command["key"] == "expiry") {
    JsonVariantConst json_expiry = command["value"].as<JsonVariantConst>();
    success = setExpiry(json_expiry);
  }
  else if (command["key"] == "promisc") {
    JsonVariantConst json_promisc = command["value"].as<JsonVariantConst>();
    success = setPromisc(json_promisc);
  }
#ifdef DYNAMIC_LOG
  else if (command["key"] == "log_level") {
    JsonVariantConst json_log_level = command["value"].as<JsonVariantConst>();
    success = setLogLevel(json_log_level);
  }
#endif
  else if (command["key"] == "clear") {
    eepromClear();
  }
  else if (command["key"] == "reset") {
    resetBoard();
  }
  else {
    LOG(ERROR, "'key' unknown");
  }

  if (success) {
    eepromWriteConfig();
    getConfig(command);
  }

  _end;
}

bool setAddress(JsonArrayConst& json_address) {
  _start("setAddress");

  bool valid = false;
  size_t address_cnt = json_address.size();
  uint8_t address[address_cnt] = {0};

  // default limit of 1 to 4 addresses
  if (!address_cnt || address_cnt > ADDRESS_MAX) {
    LOG(ERROR, "'address' must contain between 1 and %u elements", ADDRESS_MAX);

    goto end;
  }

  for (uint8_t num = 0; num < address_cnt; num++) {
    // ensure they're actually uint8_t
    if (!json_address[num].is<unsigned char>()) {
      LOG(ERROR, "'address' values must be between 0 and 255");

      goto end;
    }

    address[num] = json_address[num].as<unsigned char>();
  }

  // check for duplicates
  for (uint8_t x = 0; x < address_cnt; x++)
    for (uint8_t y = x + 1; y < address_cnt; y++)
      if (address[x] == address[y]) {
        LOG(ERROR, "'address' cannot contain duplicates");

        goto end;
      }

  // if we're here, data looks good
  valid = true;
  expireAllAlarms();

  // copy over the new addresses
  memcpy(s.address, address, address_cnt);
  s.address_cnt = address_cnt;

end:
  _end;

  return (valid);
}

bool setBaud(JsonVariantConst& json_baud) {
  _start("setBaud");

  bool valid = false;

  if (json_baud.is<unsigned long>()) {
    unsigned long baud = json_baud.as<unsigned long>();

    if (baud) {
      s.baud = baud;
      valid = true;
      initSerial();
    }
    else {
      LOG(ERROR, "'baud' must be > 0");
    }
  }
  else {
    LOG(ERROR, "'baud' must be an integer");
  }

  _end;

  return (valid);
}

bool setExpiry(JsonVariantConst& json_expiry) {
  _start("setExpiry");

  bool valid = false;

  if (json_expiry.is<unsigned short>()) {
    unsigned short expiry = json_expiry.as<unsigned short>();

    if (expiry) {
      s.expiry = expiry;
      valid = true;
    }
  }

  if (!valid)
    LOG(ERROR, "'expiry' must be an integer between 1-65535");

  _end;

  return (valid);
}

bool setPromisc(JsonVariantConst& json_promisc) {
  _start("setPromisc");

  bool valid = false;

  if (json_promisc.is<bool>()) {
    s.promisc = json_promisc.as<bool>();
    valid = true;
    expireAllAlarms();
  }
  else {
    LOG(ERROR, "'promisc' must be true or false");
  }

  _end;

  return (valid);
}

#ifdef DYNAMIC_LOG
bool setLogLevel(JsonVariantConst &json_log_level) {
  _start("setLogLevel");

  bool valid = false;

  if (json_log_level.is<const char*>()) {
    const char *level_str = json_log_level.as<const char*>();

    for (uint8_t level = 0; level < NUM_LEVELS;  level++) {
      if (!strcmp(log_strs[level], level_str)) {
        s.log_level = level;
        valid = true;
        break;
      }
    }

    if (!valid)
      LOG(ERROR, "'log_level' '%s' is unknown", level_str);
  }
  else {
    LOG(ERROR, "'log_level' must be a string");
  }

  _end;

  return (valid);
}
#endif

void setup() {
  bool crcOk = eepromReadConfig();

  initSerial();

  // so usb uart can enumerate
  if (_LOG_LEVEL_ == TRACE)
    delay(1000);

  _start("setup");

  if (!crcOk) {
    LOG(ERROR, "eeprom crc mismatch - loaded defaults");

    eepromWriteConfig(); // save defaults
  }

  LOG(INFO, "setup");

  dumpConfig();

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

  initAlarmStates();

  startRx();

  LOG(DEBUG, "entering loop");

  _end;
}

void loop() {
  uint16_t max_delay = 1000; // ~1s
  char cmd[SERIAL_BUFFER];

  // set to 8s
  updateWatchdog();

  // wait until GDO2 goes HIGH - this signifies a packet has been received
  while (digitalRead(GDO2) == LOW && !Serial.available() && max_delay) {
    delay(1);
    max_delay--;
  }

  if (digitalRead(GDO2) == HIGH)
    processRxFifo();

  if (Serial.available()) {
    memset(&cmd, 0, sizeof(cmd));
    // FIXME: if bytes come in slowly, but at a greater interval than 100ms (default SERIAL_TIMEOUT), it can cause a wdog reset
    Serial.readBytesUntil('\n', cmd, sizeof(cmd) - 1);
    parseCommand(cmd);
  }

  if (!s.promisc)
    expireAlarmStates();
}
