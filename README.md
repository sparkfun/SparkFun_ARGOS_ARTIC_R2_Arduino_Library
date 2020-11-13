SparkFun ARGOS ARTIC R2 Arduino Library
===========================================================

![SparkX ARTIC R2 Breakout]()

[*SparkX ARTIC R2 Breakout*]()

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
* **[Product Repository](https://github.com/sparkfunX/ARTIC_R2_Breakout)** - Main repository (including hardware files)

Booting
-------------------

The ARTIC R2 can either boot from on-board flash memory, or the firmware can be downloaded by the MCU via SPI.
This library supports both methods but defaults to downloading the firmware via SPI.

If you want to change this, you will need to edit [the library header file](https://github.com/sparkfun/SparkFun_ARGOS_ARTIC_R2_Arduino_Library/blob/master/src/SparkFun_ARGOS_ARTIC_R2_Arduino_Library.h#L36)
and comment the line which says ```#define ARTIC_R2_UPLOAD_FIRMWARE```. From then on the ARTIC R2 will boot from flash memory instead. The compiled code will also be much more compact,
allowing it to run on more modest MCUs.

License Information
-------------------

This product is _**open source**_!

Please review the LICENSE.md file for license information.

If you have any questions or concerns on licensing, please contact techsupport@sparkfun.com.

Please use, reuse, and modify these files as you see fit. Please maintain attribution to SparkFun Electronics and release any derivative under the same license.

Distributed as-is; no warranty is given.

- Your friends at SparkFun.
