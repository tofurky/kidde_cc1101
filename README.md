# kidde_cc1101

- [Disclaimer](#disclaimer)  
- [License](#license)  
- [Introduction](#introduction)  
- [Files](#files)  
- [Protocol](#protocol)  
- [Pinout](#pinout)  
- [Firmware](#firmware)  

## Disclaimer

None of the information/code in this repository is suitable for real-world use in life safety 
applications. It is for educational/informational purposes only.

## License
[Unlicense](https://unlicense.org/). See [UNLICENSE](/UNLICENSE). Where applicable, any copyrights 
and patents relating to the protocol itself supersede this.

## Introduction

This [repo](https://github.com/tofurky/kidde_cc1101) contains the result of my reverse engineering 
of the [433MHz](https://en.wikipedia.org/wiki/LPD433) protocol used by Kidde 
[KN-COSM-B-RF](https://www.kidde.com/home-safety/en/us/products/fire-safety/combination-smoke-co-alarms/kn-cosm-b-rf/) 
smoke/CO alarms. It is not known whether other "Wireless Interconnect" models use the same/similar 
protocol as I do not own them.

The protocol is rather simple - no encryption or obfuscation is used and there are only a few 
distinct commands. The most mysterious part of it is the register settings used in the TI CC1101 
radios. There are 40+ 8-bit registers on the CC1101, so simply guessing them seemed out of the 
question (for me, anyways).

Instead, a cheap [logic analyzer](https://sigrok.org/wiki/Fx2lafw) was connected between the 
[PIC16F883](https://www.microchip.com/wwwproducts/en/PIC16F883) and 
[CC1101](https://www.ti.com/product/CC1101) on a first-generation 
[Wink Hub](https://www.wink.com/products/wink-hub/), and 
[PulseView's](https://sigrok.org/wiki/PulseView) 
[CC1101 decoder](https://sigrok.org/wiki/Protocol_decoder:Cc1101) was used to make sense of the 
captured [SPI](https://sigrok.org/wiki/Protocol_decoder:Spi) data.

## Files

The following files are contained within this repo:
- [README.md](/README.md): this README  
- [UNLICENSE](/UNLICENSE): the (UN)LICENSE  
- [kidde_cc1101.ino](/arduino/kidde_cc1101.ino): Arduino firmware tailored to ATmega328P  
- [stm32_cc1101.ino](/stm32duino/stm32_cc1101.ino): example [STM32duino](https://github.com/stm32duino) 
sketch implementing barebones receiver, with optional test mode to transmit  
- [wink_cc1101_2020-03-07.sr](/pulseview/wink_cc1101_2020-03-07.sr): PulseView capture of PIC16F883 
initializing CC1101 during power on of Wink Hub  
- [pulseview.png](/image/pulseview.png): screenshot of said PulseView capture  
- [wink_cc1101_pic16f883_pinout.jpg](/image/wink_cc1101_pic16f883_pinout.jpg): annotated image of 
logic analyzer connection to Wink Hub, also shown in the [Pinout](#pinout) section below  
- [wink_cc1101_pic16f883_large.jpg](/image/wink_cc1101_pic16f883_large.jpg): larger image showing 
 layout of Wink Hub PCB  

## Protocol

The data coming from the CC1101 can be described like so:

```
 struct kidde_pkt {  
   uint8_t address;  
   uint8_t command;  
   uint8_t suffix[2];  
   uint8_t rssi;  
   uint8_t crc_lqi;  
 };  
```

- `address`: equivalent to the DIP switches located in the smoke detector's battery compartment
- `command`: one of _TEST_, _HUSH_, _CO_, _SMOKE_, or _BATTERY_, described in more detail below
- `suffix`: a fixed value of `0x9655` (YMMV - I only have a sample size of 1)
- `rssi`: the signal strength of the received packet
- `crc_lqi`: MSB represents CRC OK, lower 7 bits represent link quality

The `rssi` and `crc_lqi` are not sent over the air - they are appended to each packet in the RX 
FIFO by the CC1101 chip per the register settings.

The `command` field will contain one of the following:

```
 #define KIDDE_COMMAND (0x80)  
 #define KIDDE_TEST (KIDDE_COMMAND | 0x00)  
 #define KIDDE_HUSH (KIDDE_COMMAND | 0x01)  
 #define KIDDE_CO (KIDDE_COMMAND | 0x02)  
 #define KIDDE_SMOKE (KIDDE_COMMAND | 0x03)  
 #define KIDDE_BATTERY (KIDDE_COMMAND | 0x0D)  
```

In practice, all commands appear to have the MSB set (`KIDDE_COMMAND` above). However, detectors 
(and Wink) will respond to commands without the MSB set. The Wink Hub does not acknowledge 
_BATTERY_ without the MSB set.

- `KIDDE_TEST`: sent by the detector when the test button is pressed
- `KIDDE_HUSH`: sent by the detector when the test button is held while an alarm is active
- `KIDDE_CO`: sent by the detector when CO is detected
- `KIDDE_SMOKE`: sent by the detector when smoke is detected
- `KIDDE_BATTERY`: periodically sent by the detector if the battery is low

The detectors both send and receive commands. If a single detector goes off, other detectors within 
range will start alarming as well. The exception to this appears to be the _BATTERY_ command.

The _HUSH_ command, when received by an alarming detector, should silence the alarm. I had mixed 
results when attempting to silence an active alarm by transmitting this command.

A detector will only respond to commands if the `address` field matches that set by the DIP 
switches in its battery compartment.

The detector appears to be in a low-power sleep state most of the time to preserve battery life. 
As a result of this, and probably to ensure reliable transmission, an alarming detector does not 
send just a single or a few packets. Instead, over 1000 packets are sent when a detector broadcasts 
an alarm. Some rough measurement of the received data shows about 10ms delay between packets.

## Pinout

The PIC16F883 as used by the Wink Hub only has a single SPI interface, so it was rather simple to 
figure out how things were wired up. The trickiest part was trying to get two of my HP logic probes 
on neighboring pins. For one of the connections (CS), I clipped onto a resistor intead of an MCU 
lead.

![Wink Hub CC1101 pinout](/image/wink_cc1101_pic16f883_pinout.jpg?raw=true)

## Firmware

- [Background](#background)  
- [Requirements](#requirements)  
- [Configuring](#configuring)  
- [Example commands](#example-commands)  
- [Example output](#example-output)  
- [Example Home Assistant configuration](#example-home-assistant-configuration)  

### Background

Two firmwares are included in this repo - [stm32_cc1101.ino](/stm32duino/stm32_cc1101.ino), which 
is a more simple proof-of-concept that can both send and receive the Kidde protocol, and was not 
intended for much beyond that, and [kidde_cc1101.ino](/arduino/kidde_cc1101.ino), which I created 
so I could use my smoke detector with [Home Assistant](https://www.home-assistant.io/).

[kidde_cc1101.ino](/arduino/kidde_cc1101.ino) was developed using a [RM1101-USB-232](https://www.ebay.com/itm/433Mhz-CC1101-USB-Wireless-RF-Transceiver-Module-10mW-USB-UART-MAX232-RS232/311565600262) 
which unfortunately wasn't suitable in its stock form due to the SPI programming interface being 
fused off, and because the CC1101 isn't wired up to the ATmega48PA using hardware SPI pins (no idea 
why). So, I desoldered and replaced the ATmega48PA with an ATmega328P and added jumpers to connect 
the hardware SPI interface to the CC1101. Not pretty, but it works. Some pics of the before/after 
can be found [here](https://imgur.com/a/ixWAovH). Something like a [SIGNALduino](http://wiki.in-circuit.de/index.php5?title=SIGNALduino_Stick) 
would be a better route to go, though they're relatively pricey.

### Requirements

[kidde_cc1101.ino](/arduino/kidde_cc1101.ino) should work on any ATmega board >= ATmega328P (given 
RAM / flash requirements) as long as hardware SPI and GDO2 are connected to the CC1101 module. Note 
that CC1101 operates at 3.3V so the ATmega should operate at 3.3V or have appropriate level 
shifting.

The following external libraries are used:
- [ArduinoJson](https://arduinojson.org/)
- [MemoryFree](https://github.com/McNeight/MemoryFree) (optional, for debug logging)

[MiniCore](https://github.com/MCUdude/MiniCore) was originally chosen due to it supporting the 
ATmega48, however it stuck even after swapping the chip for a ATmega328P. It allows easily 
specifying things like the "non-standard" 8MHz crystal found on the RM1101-USB-232 and other 3.3V 
boards as well as BOD fuse settings. However, the standard Arduino core will work as well, but 
you might need to edit boards.txt and replace e.g. `uno.build.f_cpu=16000000L` with 
`uno.build.f_cpu=8000000L` if your board is 8MHz.

### Configuring

Interaction with [kidde_cc1101.ino](/arduino/kidde_cc1101.ino) is via JSON sent over the UART. The 
default settings are as follows:
- log level of INFO (minimal non-alarm chatter)  
- smoke detector address of 0b00000000 (factory dip switch settings, IIRC)  
- alarm expiry of 10s (JSON expiry message is sent 10s after the last packet is rx'd from detector)  
- 38400 baud (should work for both 8MHz and 16MHz boards)  

A few compile-time defines are available:
- `LOG_LEVEL` - if compiling without `DYNAMIC_LOG`, log-level is fixed at this  
- `DYNAMIC_LOG` - allows changing log level at runtime, and subsequently stores **all** log messages in flash  
- `MEMORY_USAGE` - at TRACE log-level, will output memory usage. Requires [MemoryFree](https://github.com/McNeight/MemoryFree) library  
- `ADDRESS_MAX` - maximum number of monitored addresses. Each address requires ~96 bytes of RAM. Default is 4  
- `CS`, `MOSI`, `MISO`, `SCLK`, `GDO0`, `GDO2`, `LED` - corresponding pins on ATmega. Note that there are separate sections depending on if MiniCore or standard Arduino is used  
- `RM1101_USB_232`, `EN`, `ORIG_SCLK`, `ORIG_MOSI`, `ORIG_MISO` - only relevant if you're using a modified RM1101-USB-232. EN is the amplifier enable pin  
- `SPISettings CC1101()` - if you want to use an SPI speed other than 1MHz  

### Example commands

Assuming that the connected ATmega's serial port is `/dev/ttyUSB0`:

Change the baud to 500000:  
`echo '{"type":"set","key":"baud","value":500000}' >> /dev/ttyUSB0`

Change the baud to 115200:  
`echo '{"type":"set","key":"baud","value":115200}' >> /dev/ttyUSB0`

Monitor addresses 0x00, 0xAA, 0xCC, and 0xBB:  
`echo '{"type":"set","key":"address","value":[0,170,204,187]}' >> /dev/ttyUSB0`

Monitor address 0xFF:  
`echo '{"type":"set","key":"address","value":[255]}' >> /dev/ttyUSB0`

Set alarm expiry to 60s:  
`echo '{"type":"set","key":"expiry","value":60}' >> /dev/ttyUSB0`

Set alarm expiry to 5m:  
`echo '{"type":"set","key":"expiry","value":300}' >> /dev/ttyUSB0`

Enable promiscuous mode (note: this disables normal stateful alarm functionality):  
`echo '{"type":"set","key":"promisc","value":true}' >> /dev/ttyUSB0`

Disable promiscuous mode:  
`echo '{"type":"set","key":"promisc","value":false}' >> /dev/ttyUSB0`

Set log level to TRACE (requires compiling with `DYNAMIC_LOG`):  
`echo '{"type":"set","key":"log_level","value":"trace"}' >> /dev/ttyUSB0`

Clear EEPROM (requires reset to take effect):  
`echo '{"type":"set","key":"clear"}' >> /dev/ttyUSB0`

Reset ATmega:  
`echo '{"type":"set","key":"reset"}' >> /dev/ttyUSB0`

Recover from bad baud (or just reflash with `EEPROM_MAGIC` changed to something other than `{'k', 'i', 'd', 'd', 'e'}`):  
```
stty -F /dev/ttyUSB0 115200 # this should be the baud rate you set
perl -MTime::HiRes -e '$|++; my $cmd = q|{"type":"set","key":"clear"}|; foreach my $byte (split("", $cmd)) { print $byte; Time::HiRes::sleep(0.05) } print "\n";' >> /dev/ttyUSB0`
```
Then cycle power.

### Example output

Board reset at INFO log level:
```
{"millis":0,"type":"log","level":"info","caller":"setup","msg":"setup"}
{"address":[170],"expiry":10,"baud":500000,"promisc":false}
{"millis":1007,"type":"log","level":"info","caller":"resetCC1101","msg":"reset took 2168 us"}
{"type":"chip_info","status":15,"partnum":0,"version":20}
{"millis":1013,"type":"log","level":"info","caller":"calibrateCC1101","msg":"calibration took 1072 us"}
```

Below are artificially generated alarms - the normal duration is 10s for a test, and indefinite for smoke/CO.

Smoke detected and subsequent expiry:
```
{"millis":70160,"type":"alarm","address":170,"command":"smoke","min_rssi":4,"max_rssi":4,"duration":0,"count":1,"active":true}
{"millis":98447,"type":"alarm","address":170,"command":"smoke","min_rssi":1,"max_rssi":252,"duration":17447,"count":10,"active":false}
```

CO detected and subsequent expiry:
```
{"millis":69158,"type":"alarm","address":170,"command":"co","min_rssi":3,"max_rssi":3,"duration":0,"count":1,"active":true}
{"millis":81649,"type":"alarm","address":170,"command":"co","min_rssi":3,"max_rssi":4,"duration":2472,"count":2,"active":false}
```

Test detected and subsequent expiry:
```
{"millis":67289,"type":"alarm","address":170,"command":"test","min_rssi":4,"max_rssi":4,"duration":0,"count":1,"active":true}
{"millis":91236,"type":"alarm","address":170,"command":"test","min_rssi":3,"max_rssi":4,"duration":13416,"count":11,"active":false}
```

Low battery detected and subsequent expiry:
```
{"millis":69361,"type":"alarm","address":170,"command":"battery","min_rssi":3,"max_rssi":3,"duration":0,"count":1,"active":true}
{"millis":93300,"type":"alarm","address":170,"command":"battery","min_rssi":2,"max_rssi":4,"duration":13716,"count":5,"active":false}
```

Promiscuous mode:
```
{"millis":1455659,"type":"packet","command":"hush","address":187,"raw_command":129,"suffix":38655,"rssi":13,"crc":true,"lqi":4}
{"millis":1455712,"type":"packet","command":"test","address":204,"raw_command":128,"suffix":38655,"rssi":13,"crc":true,"lqi":3}
{"millis":1455769,"type":"packet","command":"unknown","address":0,"raw_command":136,"suffix":38655,"rssi":13,"crc":true,"lqi":2}
{"millis":1455790,"type":"packet","command":"hush","address":204,"raw_command":129,"suffix":38655,"rssi":13,"crc":true,"lqi":4}
{"millis":1455800,"type":"packet","command":"unknown","address":0,"raw_command":138,"suffix":38655,"rssi":13,"crc":true,"lqi":2}
{"millis":1455822,"type":"packet","command":"battery","address":0,"raw_command":141,"suffix":38655,"rssi":13,"crc":true,"lqi":2}
{"millis":1455953,"type":"packet","command":"unknown","address":204,"raw_command":139,"suffix":38655,"rssi":13,"crc":true,"lqi":2}
{"millis":1455990,"type":"packet","command":"battery","address":170,"raw_command":141,"suffix":38655,"rssi":13,"crc":true,"lqi":2}
{"millis":1456039,"type":"packet","command":"unknown","address":170,"raw_command":135,"suffix":38655,"rssi":13,"crc":true,"lqi":0}
{"millis":1456105,"type":"packet","command":"battery","address":170,"raw_command":141,"suffix":38655,"rssi":12,"crc":true,"lqi":0}
{"millis":1456144,"type":"packet","command":"smoke","address":204,"raw_command":131,"suffix":38655,"rssi":13,"crc":true,"lqi":1}
```

### Example Home Assistant configuration

`configuration.yaml`:
```
...
sensor: !include sensor.yaml
binary_sensor: !include binary_sensor.yaml
...
```

`sensor.yaml`:
```
- platform: serial
  serial_port: /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
  baudrate: 500000
  name: kidde serial
```

`binary_sensor.yaml`:
```
- platform: template
  sensors:
    kidde_battery:
      friendly_name: kidde battery
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "battery" %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_battery', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}

    kidde_co:
      friendly_name: kidde co
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "co" %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_co', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}

    kidde_hush:
      friendly_name: kidde hush
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "hush" %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_hush', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}

    kidde_smoke:
      friendly_name: kidde smoke
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "smoke" %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_smoke', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}

    kidde_test:
      friendly_name: kidde test
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "test" %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_test', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}

    kidde_unknown:
      friendly_name: kidde unknown
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "unknown" %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_unknown', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}
```

If you have multiple detectors on different addresses (note - this will effectively disable the wireless interconnect between them), you could filter by address like so:
```
    kidde_smoke_garage:
      friendly_name: kidde smoke (garage - 0x00)
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "smoke" and state_attr('sensor.kidde_serial', 'address') == 0 %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_smoke_garage', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}

    kidde_smoke_gazebo:
      friendly_name: kidde smoke (gazebo - 0xAA)
      value_template: >-
        {% if state_attr('sensor.kidde_serial', 'command') == "smoke" and state_attr('sensor.kidde_serial', 'address') == 170 %}
          {{ state_attr('sensor.kidde_serial', 'active') }}
        {% else %}
          {{ is_state('binary_sensor.kidde_smoke_gazebo', 'on') }}
        {% endif %}
      availability_template: >-
        {% if not is_state('sensor.kidde_serial', 'unavailable') %}
          true
        {% endif %}
```
