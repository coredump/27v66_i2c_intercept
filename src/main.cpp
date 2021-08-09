/* The intent here is to add something  between the MICOM and Jungle chips on a
  Sony 27V66 CRT TV, and intercept the i2c commands that Sony uses to disable
  the secondary RGB input on the Jungle so we can inject our own signal there.
  Board used: STM32F303CCT6 Blackpill by Robotdyn
    https://stm32-base.org/boards/STM32F103C8T6-RobotDyn-Black-Pill
  All the i2c communication and registers were lifted from the CXA2061S
  datasheet
    https://www.alldatasheet.com/datasheet-pdf/pdf/46750/SONY/CXA2061S.html
  The TV actually uses a different chip (CXA2133S) but the 2061S seems to be at
  least 90% compatible and the i2c info there worked.
*/
#include <Arduino.h>
#include <Wire.h>
#include <singleLEDLibrary.h>

/* We use this circular buffer to be able to have really fast i2c slave
   handling functions (since they are actually interrupt handlers). That
   way we can receive all data and add to the buffer and then deal with it
   when the MCU isn't handling some other i2c data
*/
#define CIRCULAR_BUFFER_INT_SAFE
#include <CircularBuffer.h>

// Those are just for convenience since I forget the pins all the time
#define MASTER_SCL PB6
#define MASTER_SDA PB7
#define SLAVE_SCL PA9
#define SLAVE_SDA PA10

TwoWire Master = TwoWire(MASTER_SDA, MASTER_SCL);
TwoWire Slave = TwoWire(SLAVE_SDA, SLAVE_SCL);
sllib led1(LED_BLUE);
CircularBuffer<byte, 64> sBuf;

/* This is a heavy handed solution for the problem with Micom reads.
   Every few seconds the Micom reads the status from the Jungle chip
   to make sure the TV isn't on fire or something. If those tests fail
   it shuts off the cannon and turns off the TV. So instead of reading
   this status from the jungle and sending (that takes forever) we will
   just send the status and hope the TV isn't actually on fire.
*/
volatile byte mRCount = 0;
byte dataInit[] = {0x00, 0x85};
byte dataOK[] = {0x40, 0x85};

void slaveReceive(int n);
void slaveRequest();

void setup() {
  Master.begin();
  Slave.begin(0x44);
  Slave.onReceive(slaveReceive);
  Slave.onRequest(slaveRequest);

  /* Since we are using a 3.3V MCU that has 5V tolerant Pins, we don't
  want the Pullups to be enabled. This seem to be the way to do it.
  */
  digitalWrite(MASTER_SDA, LOW);
  digitalWrite(MASTER_SCL, LOW);
  digitalWrite(SLAVE_SDA, LOW);
  digitalWrite(SLAVE_SCL, LOW);

  // Put a heartbeat led to make things easier to see
  led1.setBreathSingle(1000);
}

/* Those two functions are ISRs so be fast and follow all
   ISR tips and tricks
*/
void slaveReceive(int n) {
  while (Slave.available()) {
    byte c = Slave.read();
    sBuf.push(c);
  }
}

/* During my signal analysis of the i2c protocol that this TV uses, I verified
   that usually takes around 6 requests for the IKR bit to stabilize and flip
   (what finishes the startup self check). So we literally count how many times
   we got asked for the register and then send the OK looking data back.
   Ugly but it works.
*/
void slaveRequest() {
  if (mRCount > 7) {
    Slave.write(dataInit, 2);
  } else {
    Slave.write(dataOK, 2);
  }
}

void loop() {
  byte bSize = sBuf.size(); // if there's data from the micom, this will be > 0
  if (bSize > 0) {
    byte data[bSize];
    byte i = 0;

    while (!sBuf.isEmpty()) {
      byte d = sBuf.shift();
      data[i] = d;
      ++i;
    }

    /* The Micom writes to the Jungle in groups of three bytes/registers at a
       time, so the register that we want is actually on 0xA that is included
       on the group starting on 0x9. We use a & to mask and change only the
       last bit to 0 and enable the RGB SEL bit.
       Also, theoretically using a SWITCH with ascending int cases makes
       the MCU spend less cycles, so why the heck not.
    */
    switch (data[0]) {
    case 0x9:
      data[2] = (data[2] & 0xfe);
      break;

    /* In didn't see this being called directly, but just in case it happens
       sometime, we also mask the RGB SEL bit if we receive any data for the 0XA
       register
    */
    case 0xA:
      data[1] = (data[1] & 0xfe);
      break;

    default:
      break;
    }

    Master.beginTransmission(0x44);
    Master.write(data, bSize);
    Master.endTransmission();
  }

  led1.update();
}