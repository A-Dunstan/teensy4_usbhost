This is a USB Host library (based on the EHCI specification) specifically aimed at supporting the Teensy 4.x (NXP IMXRT1062) hardware.

The core EHCI code (found in src/ehci/) is aimed to be platform specific. It contains full support for low/full/high speed devices, including control/bulk/interrupt and isochronous endpoints. Basic support for Hubs is also included and requires no extra setup; Hub drivers with be instantiated as required.

Device classes are supported by implementing a driver for them, which will either claim the whole device or a specific interface of the device. Usercode communicates with devices by calling driver functions, which in turn issue standard USB requests to endpoints on the device/interface.

Full hotplugging support is included.

Teensy specific details:
- any type of memory (DTCM/DMAMEM/EXTMEM) should work correctly for any purpose, including the host and driver instances. However attention should be paid to alignment of cached memory.
- several example device drivers can be found in the src/drivers/ directory. Most of these have specific example sketches to demonstrate their usage.
- detailed error codes are usually returned via errno.
- either or both USB ports can be used in host mode, using the TeensyUSBHost1/TeensyUSBHost2 classes.
- this library relies on TeensyAtomThreads to provide multithreading capabilities. TeensyAtomThreads requires no extra initialization steps and will operate transparently to the regular Teensy core code if the user doesn't wish to make use of its functions.
