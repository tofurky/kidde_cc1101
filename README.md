# kidde_cc1101

- [Disclaimer](#disclaimer)  
- [License](#license)  
- [Introduction](#introduction)  
- [Files](#files)  
- [Protocol](#protocol)  
- [Pinout](#pinout)  

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

