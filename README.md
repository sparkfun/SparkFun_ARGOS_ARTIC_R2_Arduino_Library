SparkFun ARGOS ARTIC R2 Arduino Library
===========================================================

[![ARGOS ARTIC R2 Shield](https://cdn.sparkfun.com//assets/parts/1/6/2/1/7/17236-Artic_R2_Breakout-01a.jpg)](https://www.sparkfun.com/products/17236)

[*ARGOS ARTIC R2 Shield*](https://www.sparkfun.com/products/17236)

The ARTIC R2 chipset allows you to send and receive short data messages to and from anywhere via the ARGOS satellite network. Dedicated to programs which are related to environmental protection,
awareness or study, or to protecting human life, the ARGOS constellation provides full global coverage including the polar regions.

The ARTIC is an integrated low power small size ARGOS 2/3/4 single chip radio. ARTIC implements a message based wireless interface. For satellite uplink communication, ARTIC will encode, modulate and transmit
provided user messages. For downlink communication, ARTIC will lock to the downstream, demodulate and decode and extract the satellite messages.

The ARTIC can transmit signals in frequency bands around 400MHz and receive signals in the bands around 466MHz, in accordance with the ARGOS satellite system specifications.

The ARTIC is compliant and certified for all ARGOS 2 and ARGOS 3 TX standards. It contains a RF transceiver, a frequency synthesiser and a digital baseband modem.

This library provides access to the ARTIC via its SPI interface. Full support is provided for both standard mode and burst mode communication.

Library written by Paul Clark ([SparkFun](http://www.sparkfun.com)).

Repository Contents
-------------------

* [**/examples**](./examples) - Example sketches for the library (.ino). Run these from the Arduino IDE.
* [**/src**](./src) - Source files for the library (.cpp, .h).
* [**keywords.txt**](./keywords.txt) - Keywords from this library that will be highlighted in the Arduino IDE.
* [**library.properties**](./library.properties) - General library properties for the Arduino package manager.
* [**CONTRIBUTING.md**](./CONTRIBUTING.md) - Guidance on how to contribute to this library.

Documentation
--------------

* **[Installing an Arduino Library Guide](https://learn.sparkfun.com/tutorials/installing-an-arduino-library)** - Basic information on how to install an Arduino library.
* **[ARGOS ARTIC R2 Shield Product Repository](https://github.com/sparkfunX/ARGOS-ARTIC-R2-Shield)** - Main repository (including hardware files)
* **[IOTA Product Repository](https://github.com/sparkfunX/IOTA-ARTIC-R2-Module)**
* **[smôl ARTIC R2 Product Repository](https://github.com/sparkfunX/SparkX_smol_ARTIC_R2)**

v1.1
-------------------

From v1.1.0 of the library:

We were instructed by Kineis to ensure that the Platform ID was written directly into each module
and not stored in a configuration file accessible to standard users. To comply with this, SparkFun
ARTIC R2 boards are now shipped with the Platform ID programmed into PMEM. When you buy a board from SparkFun,
it will come with a card showing what the Platform ID for that board is. You will need to contact CLS or
Woods Hole Group to have the Platform ID added to your account.

The library functions have been modified so that the Platform ID is read from PMEM instead of being a passed as an argument.
The library examples and functions read the Platform ID from the ARTIC R2 memory using ```readPlatformID```.
Early boards will return a value of zero, later boards will return an ID starting with (e.g.) 0x3368.
If the example stalls because the Platform ID is zero, you can:
- use the Arduino IDE Library Manager to install v1.0.9 of the library (which does allow the Platform ID to be passed as an argument)
- manually download and install the library using [this link to the v1.0.9 zip file](https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/archive/refs/tags/v1.0.9.zip)
or [this link to the v1.0.9 tar.gz file](https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/archive/refs/tags/v1.0.9.tar.gz)

Booting
-------------------

The ARTIC R2 can either boot from on-board flash memory, or the firmware can be downloaded by the MCU via SPI.
This library supports both methods but defaults to booting from on-board flash.

If you want to change this, you will need to edit [the library header file](https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/blob/master/src/SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h#L44)
and uncomment the line which says ```#define ARTIC_R2_UPLOAD_FIRMWARE```. From then on the ARTIC R2 will be booted via SPI instead.

**Note: the ARTIC006 firmware occupies 127KBytes of program memory. You will need an MCU with adequate memory if you choose to boot via SPI.**

License Information
-------------------

This product is _**open source**_!

Please review the LICENSE.md file for license information.

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release any derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
