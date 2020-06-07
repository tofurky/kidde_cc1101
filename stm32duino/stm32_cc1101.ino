// DISCLAIMER: This is not intended to be used for any real-world safety applications - it is for educational/informational purposes only! Use at your own risk.

// developed/tested on "blue pill" with novate cc1101 module
// linked with "Newlib Standard" for sprintf
// at least SPI (and probably USB serial) code will need to be reworked for use with an arduino core besides stm32duino

#include <stdio.h>
#include <SPI.h>

#define SPI_MOSI1_PIN PA7
#define SPI_MISO1_PIN PA6
#define SPI_SCK1_PIN PA5
#define SPI_NSS1_PIN PA4

#define CC1101_GDO0_PIN PA3
#define CC1101_GDO2_PIN PA2

// TEST_MODE will trigger a test on any smoke alarms (or wink hubs) in range.
// note that the 433MHz band only allows transmission for 2 seconds per hour in the US
// the bit rate per SmartRF Studio is ~100kbaud, so this should comply with said limits
// each packet is 4 bytes + overhead
// the *alarms themselves* are excluded for the duration of their alarm period
// in other words, tread carefully :-)
//#define TEST_MODE
#define TEST_ADDRESS_MIN 0x00
#define TEST_ADDRESS_MAX 0x00
#define TEST_COUNT 1000
#define TEST_DELAY_MIN_MS 10
#define TEST_DELAY_MAX_MS TEST_DELAY_MIN_MS * 2
#define TEST_SLEEP_MIN_S 3600
#define TEST_SLEEP_MAX_S TEST_SLEEP_MIN_S * 2
#define TEST_COMMAND KIDDE_TEST

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

#define BUFFER_LENGTH 128

char buffer[BUFFER_LENGTH];
byte reg_val;

// obtained via pulseview + cc1101 decoder on wink hub 1
// Burst write: IOCFG2 (00) = 07 2E 04 07 D3 91 04 0C 44 00 00 06 00 10 AA 80 4B F9 03 22 F7 46 07 0F 18 1D 1C C7 00 B2 30 AA 69 B6 10 EA 2A 00
uint8_t kidde_regs[] = {
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

void setup() {
  unsigned long start_time;

  // USB CDC ACM
  Serial.begin(115200);
  // wait until something (e.g. miniterm) attaches
  while (!Serial);
  Serial.println("setup()");

  // Asserts when the RX FIFO has overflowed. De-asserts when the FIFO has been flushed.
  pinMode(CC1101_GDO0_PIN, INPUT);
  digitalWrite(CC1101_GDO0_PIN, LOW);
  // Asserts when a packet has been received with CRC OK. De-asserts when the first byte is read from the RX FIFO.
  pinMode(CC1101_GDO2_PIN, INPUT);
  digitalWrite(CC1101_GDO2_PIN, LOW);

  SPI.begin(SPI_NSS1_PIN);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  // reset cc1101
  Serial.println("issuing SRES");
  start_time = micros();
  SPI.transfer(SPI_NSS1_PIN, CC1101_STROBE_SRES);
  // cc1101 re-uses this to signal readiness after certain commands
  pinMode(SPI_MISO1_PIN, INPUT);
  digitalWrite(SPI_MISO1_PIN, HIGH);
  while (digitalRead(SPI_MISO1_PIN) == HIGH);
  snprintf(buffer, BUFFER_LENGTH, "reset took %lu us\n", micros() - start_time);
  Serial.print(buffer);

  // get chip information
  SPI.transfer(SPI_NSS1_PIN, CC1101_STROBE_SNOP, SPI_CONTINUE);
  reg_val = SPI.transfer(SPI_NSS1_PIN, 0, SPI_LAST);
  snprintf(buffer, BUFFER_LENGTH, "CC1101 status is: 0x%02hhx\n", reg_val);
  Serial.print(buffer);
  SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_READ_BURST | CC1101_REG_PARTNUM, SPI_CONTINUE);
  reg_val = SPI.transfer(SPI_NSS1_PIN, 0, SPI_LAST);
  snprintf(buffer, BUFFER_LENGTH, "CC1101 partnum is: 0x%02hhx\n", reg_val);
  Serial.print(buffer);
  SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_READ_BURST | CC1101_REG_VERSION, SPI_CONTINUE);
  reg_val = SPI.transfer(SPI_NSS1_PIN, 0, SPI_LAST);
  snprintf(buffer, BUFFER_LENGTH, "CC1101 version is: 0x%02hhx\n", reg_val);
  Serial.print(buffer);

  // burst write regs for kidde receiver
  Serial.println("writing CC1101 regs");
  SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_WRITE_BURST | CC1101_REG_IOCFG2, SPI_CONTINUE);
  for (uint8_t reg = 0; reg < sizeof(kidde_regs); reg++) {
    SPI.transfer(SPI_NSS1_PIN, kidde_regs[reg], (reg == sizeof(kidde_regs) - 1 ? SPI_LAST : SPI_CONTINUE));
    snprintf(buffer, BUFFER_LENGTH, "0x%02hhx <- 0x%02hhx\n", reg, kidde_regs[reg]);
    Serial.print(buffer);
  }

  // burst read regs back to verify
  Serial.println("reading CC1101 regs");
  SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_READ_BURST | CC1101_REG_IOCFG2, SPI_CONTINUE);
  for (uint8_t reg = 0; reg < sizeof(kidde_regs); reg++) {
    reg_val = SPI.transfer(SPI_NSS1_PIN, 0, (reg == sizeof(kidde_regs) - 1 ? SPI_LAST : SPI_CONTINUE));
    snprintf(buffer, BUFFER_LENGTH, "0x%02hhx -> 0x%02hhx\n", reg, reg_val);
    Serial.print(buffer);
  }

  // frequency synthesizer calibration
  Serial.println("issuing SCAL");
  unsigned long cal_start = micros();
  SPI.transfer(SPI_NSS1_PIN, CC1101_STROBE_SCAL);
  while (SPI.transfer(SPI_NSS1_PIN, CC1101_STROBE_SNOP) & CC1101_STATUS_CAL);
  snprintf(buffer, BUFFER_LENGTH, "calibration took %lu us\n", micros() - cal_start);
  Serial.print(buffer);

  Serial.println("setting GDO2 cfg");
  SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_WRITE | CC1101_REG_IOCFG2, SPI_CONTINUE);
  SPI.transfer(SPI_NSS1_PIN, 0x07, SPI_LAST);

#ifndef TEST_MODE
  Serial.println("issuing SRX");
  SPI.transfer(SPI_NSS1_PIN, CC1101_STROBE_SRX);
#else
  Serial.println("issuing STX");
  SPI.transfer(SPI_NSS1_PIN, CC1101_STROBE_STX);
#endif

  Serial.println("entering loop()");
}

#ifndef TEST_MODE
void loop() {
  kidde_pkt pkt;
  char kidde_cmd_str[16];

  // wait until GDO2 goes HIGH - this signifies a packet has been received
  while (digitalRead(CC1101_GDO2_PIN) == LOW);

  SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_READ_BURST | CC1101_REG_RXBYTES, SPI_CONTINUE);
  reg_val = SPI.transfer(SPI_NSS1_PIN, 0, SPI_LAST);
  snprintf(buffer, BUFFER_LENGTH, "got packet. rx fifo has %u bytes\n", reg_val);
  Serial.print(buffer);

  reg_val = SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_READ_BURST | CC1101_RX_FIFO, SPI_CONTINUE);
  pkt.address = SPI.transfer(SPI_NSS1_PIN, 0, SPI_CONTINUE);
  pkt.command = SPI.transfer(SPI_NSS1_PIN, 0, SPI_CONTINUE);
  pkt.suffix[0] = SPI.transfer(SPI_NSS1_PIN, 0, SPI_CONTINUE);
  pkt.suffix[1] = SPI.transfer(SPI_NSS1_PIN, 0, SPI_CONTINUE);
  pkt.rssi = SPI.transfer(SPI_NSS1_PIN, 0, SPI_CONTINUE);
  pkt.crc_lqi = SPI.transfer(SPI_NSS1_PIN, 0, SPI_LAST);

  switch (pkt.command) {
    case KIDDE_TEST:
      strcpy(kidde_cmd_str, "test mode");
      break;
    case KIDDE_HUSH:
      strcpy(kidde_cmd_str, "hush mode");
      break;
    case KIDDE_CO:
      strcpy(kidde_cmd_str, "co detected");
      break;
    case KIDDE_SMOKE:
      strcpy(kidde_cmd_str, "smoke detected");
      break;
    case KIDDE_BATTERY:
      strcpy(kidde_cmd_str, "low battery");
      break;
    default:
      strcpy(kidde_cmd_str, "unknown");
      break;
  }

  snprintf(buffer, BUFFER_LENGTH, "millis=%lu, address=0x%02hhx, command=0x%02hhx, suffix=0x%02hhx%02hhx, rssi=%u, crc=%u, lqi=%u: %s\n", millis(), pkt.address, pkt.command, pkt.suffix[0], pkt.suffix[1], pkt.rssi, (pkt.crc_lqi >> 7), (pkt.crc_lqi & ~0x80), kidde_cmd_str);
  Serial.print(buffer);
}
#else
void loop() {
  unsigned long rand_delay;

  for(uint8_t address = TEST_ADDRESS_MIN; address <= TEST_ADDRESS_MAX; address++) {
    snprintf(buffer, BUFFER_LENGTH, "sending for address=%i\n", address);
    Serial.print(buffer);

    // the smoke alarm seems to be asleep for a lot of the time, hence the large number of packets sent.
    // this roughly mimicks what the smoke alarms themselves send (more than 1000) including the ~10ms delay
    for(uint32_t cnt = 0; cnt < TEST_COUNT; cnt++) {
      SPI.transfer(SPI_NSS1_PIN, CC1101_STROBE_STX);

      SPI.transfer(SPI_NSS1_PIN, CC1101_CMD_WRITE_BURST | CC1101_TX_FIFO, SPI_CONTINUE);
      SPI.transfer(SPI_NSS1_PIN, address, SPI_CONTINUE);
      SPI.transfer(SPI_NSS1_PIN, TEST_COMMAND, SPI_CONTINUE);
      SPI.transfer(SPI_NSS1_PIN, 0x96, SPI_CONTINUE);
      SPI.transfer(SPI_NSS1_PIN, 0x55, SPI_LAST);

      rand_delay = random(TEST_DELAY_MIN_MS, TEST_DELAY_MAX_MS);
      delay(rand_delay);
    }

    rand_delay = random(TEST_SLEEP_MIN_S, TEST_SLEEP_MAX_S);
    snprintf(buffer, BUFFER_LENGTH, "sleeping for %lus\n", rand_delay);
    Serial.print(buffer);
    delay(rand_delay * 1000);
  }

  Serial.println("done - halting.");
  while(1);
}
#endif
