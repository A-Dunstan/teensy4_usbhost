#include <teensy4_usbhost.h>

#ifndef CDC2_STATUS_INTERFACE
#error compile with USB type set to Dual Serial
#endif

DMAMEM static TeensyUSBHost2 usb;
DMAMEM static ch341::serial board;

static uint32_t baud = 115200;
static bool dtr = false;
static bool rts = false;

FLASHMEM void setup() {
  Serial.begin(0);
  SerialUSB1.begin(0);

  if (CrashReport) {
    while (!Serial);
    Serial.print(CrashReport);
    Serial.println("Press Enter to continue");
    while (Serial.read() != '\n');
  }

  usb.begin();
  board.begin(baud, SERIAL_8N1, false);
  board.set_dtr(dtr);
  board.set_rts(rts);
}

void loop() {
  if (SerialUSB1.dtr() != dtr) {
    dtr = SerialUSB1.dtr();
    board.set_dtr(dtr);
    Serial.print("DTR set to ");
    Serial.println(dtr ? "ON":"OFF");
  }
  if (SerialUSB1.rts() != rts) {
    rts = SerialUSB1.rts();
    board.set_rts(rts);
    Serial.print("RTS set to ");
    Serial.println(rts ? "ON":"OFF");
  }
  if (baud != SerialUSB1.baud()) {
    baud = SerialUSB1.baud();
    board.begin(baud, SERIAL_8N1, false);
    Serial.print("BAUD set to ");
    Serial.println(baud);
  }

  if (board.available())
    SerialUSB1.write(board.read());

  if (SerialUSB1.available())
    board.write(SerialUSB1.read());
}
