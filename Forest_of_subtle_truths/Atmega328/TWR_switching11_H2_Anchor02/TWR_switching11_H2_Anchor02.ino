/*  Compiled in Arduino 1.8.9 with modification
    Arduino pro promini 3.3V 8Mhz

    Gets order from Master to my_address 1, 2
   Alternates mode between initiator and responder every 1sec.



    Data Structure
   https://docs.google.com/spreadsheets/d/1Cie-AcoIuojLqRC60rg6zebejEXcjnMsB474eh22s5Q/edit?usp=sharing
*/

#include <SPI.h>
#include <DW1000Ng.hpp>
#include <DW1000NgUtils.hpp>
#include <DW1000NgRanging.hpp>
#include <DW1000NgTime.hpp>
#include <DW1000NgConstants.hpp>

#define my_address  103


// 101 ~112

// serial message flag
#define SET_DEFAULT_VALUE 0
#define SEND_SONG_INFO 1
#define FINAL_RESULT 2
#define EXCHANGE_FINISH 3

//wireless message flag
#define EXCHANGE_FINISHED 104

byte network_id, total_num_headphones, poll_ack_check_threshold;
//#define my_address 1
//#define poll_ack_check_threshold 4//////////////////////////11ms! total around 270ms.
#define  resetPeriod 20 ///////////////////////////////// 8ms. is the minimum
#define r_resetPeriod 20
//#define total_num_headphones 20


byte i_destination_address = 1;
byte r_destination_address = 1;

#define master_address 255

//#define poll_num 2



#define LEN_DATA 24
byte d_data[LEN_DATA];
byte i_data[LEN_DATA];
byte r_data[LEN_DATA];
byte distance_result[20];  // buffer to store ranging data
byte failure_counter[20];
byte _info[80]; // _info[0] song_num _info[1] MSB_duation  _info[2] LSB_duation  _info[3] bow_state

//#define switchingPeriod 1000


//#define default_mode 0
#define initiator_mode 1
#define responder_mode 2
#define exchange_finish_mode 4

int32_t lasttimesent = 0;


byte my_mode, poll_count;
boolean ok_send_poll, received_poll_ack = false;

// connection pins
const uint8_t PIN_RST = 3; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = 10; // spi select pin

// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile boolean receivedMessage = false;
boolean protocolFailed = false;

// timestamps to remember
uint64_t timePollSent;
uint64_t timePollAckReceived;
uint64_t timeRangeSent;

//responder only
uint64_t timePollReceived;
uint64_t timePollAckSent;
uint64_t timeRangeReceived;

// watchdog and reset period
uint32_t lastActivity , time_poll_sent, time_stamp;


// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;

device_configuration_t DEFAULT_CONFIG = {
  false,
  true,
  true,
  true,
  false,
  SFDMode::STANDARD_SFD,
  Channel::CHANNEL_5,
  DataRate::RATE_850KBPS,
  PulseFrequency::FREQ_16MHZ,
  PreambleLength::LEN_256,
  PreambleCode::CODE_3
};

interrupt_configuration_t DEFAULT_INTERRUPT_CONFIG = {
  true,
  true,
  true,
  false,
  true
};
void setup() {
  pinMode(13, OUTPUT);
  // put your setup code here, to run once:
  // DEBUG monitoring
  //Serial.begin(57600);
  //Serial.begin(115200);
  delay(2000);
  /*
    while (!(Serial.available() > 5));
    byte first_byte = Serial.read();
    byte second_byte = Serial.read();
    // my_address, network_id, total_num_headphones, poll_ack_check_threshold;
    if (first_byte == 255 && second_byte == SET_DEFAULT_VALUE) {
    my_address = Serial.read();
    network_id = Serial.read();
    total_num_headphones = Serial.read();
    poll_ack_check_threshold = Serial.read();
    } else {
    Serial.print("Problem!");
    while (1) {
      pinMode(13, OUTPUT);
      digitalWrite(13, 1);
      delay(100);
      digitalWrite(13, 0);
      delay(100);
    }

    }
  */
  
  network_id = 10;
  total_num_headphones = 12;
  poll_ack_check_threshold = 4;
  
  // Serial.println(F("### DW1000Ng-arduino-ranging-Initiator ###"));
  // initialize the driver
  DW1000Ng::initialize(PIN_SS, PIN_IRQ, PIN_RST);
  // Serial.println("DW1000Ng initialized ...");
  // general configuration
  DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
  DW1000Ng::applyInterruptConfiguration(DEFAULT_INTERRUPT_CONFIG);

  DW1000Ng::setDeviceAddress(my_address);
  DW1000Ng::setNetworkId(network_id);
  DW1000Ng::setAntennaDelay(16436);

  DW1000Ng::setTXPower(522133279);  // 0x1F1F1F1F
  // Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  /*
    delay(1000);
    char msg[128];
    DW1000Ng::getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000Ng::getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
  */

  // attach callback for (successfully) sent and received messages
  DW1000Ng::attachSentHandler(handleSent);
  DW1000Ng::attachReceivedHandler(handleReceived);
  // anchor starts by transmitting a POLL message
  receiver();
  noteActivity();
  initializeDataBuffer();
  //my_mode = default_mode;
  my_mode = responder_mode;
}
void initializeDataBuffer() {
  for (int i = 0; i < LEN_DATA; i++) {
    d_data[i] = 0;
    i_data[i] = 0;
    r_data[i] = 0;
  }
  for (int i = 0; i < 80; i++) {
    _info[i] = 0;
  }
  for (int i = 0; i < 20; i++) {
    failure_counter[i] = 0;
  }
}
void noteActivity() {
  // update activity timestamp, so that we do not reach "resetPeriod"
  lastActivity = millis();
}


void r_resetInactive() {
  // anchor listens for POLL
  expectedMsgId = POLL;
  receiver();
  noteActivity();
  //Serial.println("resetInactive");
}


void handleSent() {
  // status change on sent success
  sentAck = true;
}

void handleReceived() {
  // status change on received success
  receivedAck = true;
  //receivedMessage = true;
}
void receiver() {
  DW1000Ng::forceTRxOff();
  // so we don't need to restart the receiver manually
  DW1000Ng::startReceive();
  //  Serial.println("startReceive");
}
void transmitPoll(byte address) {
  i_data[0] = POLL;
  i_data[16] = my_mode; // initiator_mode
  i_data[17] = address;
  i_data[18] = my_address;
  i_data[19] = 0; // _message 0, in case
  DW1000Ng::setTransmitData(i_data, LEN_DATA);
  DW1000Ng::startTransmit();
  // Serial.print("TransmitPoll to"); Serial.println(address);
}
void transmitRangeReport(float curRange) {
  r_data[0] = RANGE_REPORT;
  r_data[16] = my_mode; // responder_mode
  r_data[17] = r_destination_address;
  r_data[18] = my_address;
  r_data[19] = 0; // _message 0, in case

  // add info
  /////////////////////////////////
  r_data[20] = _info[4 * (my_address - 1)];
  r_data[21] = _info[4 * (my_address - 1) + 1];
  r_data[22] = _info[4 * (my_address - 1) + 2];
  r_data[23] = _info[4 * (my_address - 1) + 3];
  /////////////////////////////////

  // write final ranging result
  memcpy(r_data + 1, &curRange, 4);
  DW1000Ng::setTransmitData(r_data, LEN_DATA);
  DW1000Ng::startTransmit();
  // Serial.println("transmitRangeReport");
}

void transmitRangeFailed() {
  r_data[0] = RANGE_FAILED;
  r_data[16] = my_mode; // responder_mode
  r_data[17] = r_destination_address;
  r_data[18] = my_address;
  r_data[19] = 0; // _message 0, in case
  DW1000Ng::setTransmitData(r_data, LEN_DATA);
  DW1000Ng::startTransmit();
  //Serial.println("transmitRangeFailed");
}


void transmitPollAck() {
  r_data[0] = POLL_ACK;
  r_data[16] = my_mode; // responder_mode
  r_data[17] = r_destination_address;
  r_data[18] = my_address;
  r_data[19] = 0; // _message 0, in case
  DW1000Ng::setTransmitData(r_data, LEN_DATA);
  DW1000Ng::startTransmit();
  // Serial.println("transmitPollAck");
}


void transmitRange() {
  i_data[0] = RANGE;
  i_data[16] = my_mode; // initiator_mode
  i_data[17] = i_destination_address;
  i_data[18] = my_address;
  i_data[19] = 0; // _message 0, in case


  /* Calculation of future time */
  byte futureTimeBytes[LENGTH_TIMESTAMP];

  timeRangeSent = DW1000Ng::getSystemTimestamp();
  timeRangeSent += DW1000NgTime::microsecondsToUWBTime(replyDelayTimeUS);
  DW1000NgUtils::writeValueToBytes(futureTimeBytes, timeRangeSent, LENGTH_TIMESTAMP);
  DW1000Ng::setDelayedTRX(futureTimeBytes);
  timeRangeSent += DW1000Ng::getTxAntennaDelay();

  DW1000NgUtils::writeValueToBytes(i_data + 1, timePollSent, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(i_data + 6, timePollAckReceived, LENGTH_TIMESTAMP);
  DW1000NgUtils::writeValueToBytes(i_data + 11, timeRangeSent, LENGTH_TIMESTAMP);
  DW1000Ng::setTransmitData(i_data, LEN_DATA);
  DW1000Ng::startTransmit(TransmitMode::DELAYED);
  //  Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
  // Serial.println("TransmitRange");

}

void printResult() {
  Serial.print("elapsed time: ");
  Serial.println((millis() - time_stamp));
  for (int i = 0; i < total_num_headphones; i++) {
    Serial.print(distance_result[i]);
    Serial.print(", ");
  }
  Serial.println();
}

void sendDistanceData() {

  byte elapsed_time = (millis() - time_stamp);

  byte s_data[23] = {255, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0};
  s_data[1] = FINAL_RESULT;
  memcpy(&s_data[2], distance_result, 20);
  s_data[22] = elapsed_time;
  Serial.write(s_data, 23);

}

void sendResult() {
  byte elapsed_time = (millis() - time_stamp);


  byte lenData = 104;
  byte s_data[lenData];
  s_data[0] = 255;

  s_data[1] = FINAL_RESULT;

  memcpy(&s_data[2], distance_result, 20);

  s_data[22] = elapsed_time;

  memcpy(&s_data[23], _info, 80);

  // get LSB of sum of s_data[2]~s_data[102]
  byte sum = 0;
  for (int i = 2; i < 103; i++) {
    sum += s_data[i];
  }
  s_data[103] = sum % 255;

  Serial.write(s_data, lenData);

}



void loop() {


 // while (my_mode == initiator_mode)twrInitiator();
  while (my_mode == responder_mode)twrResponder();

}

void twrResponder() {
  int32_t curMillis = millis();

  if (!sentAck && !receivedAck) {

    ////////////////////
   // getSerialData();
    ////////////////////

    // check if inactive
    if (curMillis - lastActivity > r_resetPeriod) {
      r_resetInactive();
    }
    return;
  }


  // continue on any success confirmation
  if (sentAck) {
    sentAck = false;
    byte msgId = r_data[0];
    if (msgId == POLL_ACK) {
      timePollAckSent = DW1000Ng::getTransmitTimestamp();
      noteActivity();
    }
    DW1000Ng::startReceive();
  }
  if (receivedAck) {
    receivedAck = false;
    // get message and parse
    DW1000Ng::getReceivedData(r_data, LEN_DATA);
    byte msgId = r_data[0];
    byte mode_from_sender = r_data[16];
    byte to_address = r_data[17];
    byte from_address = r_data[18];
    byte _message = r_data[19];

    ////////
    if (to_address != my_address || from_address == master_address) {
      r_resetInactive();
      noteActivity();
      return;
    }
    /////////

    //////////// if it's from the sender whose mode is initiator and it's for me
    if ( mode_from_sender == initiator_mode ) {
      //////////////////// send it back to where it's from
      r_destination_address = from_address;
      ////////////////////
      if (msgId != expectedMsgId) {
        // unexpected message, start over again (except if already POLL)
        protocolFailed = true;
      }
      if (msgId == POLL) {
        // on POLL we (re-)start, so no protocol failure
        protocolFailed = false;
        timePollReceived = DW1000Ng::getReceiveTimestamp();
        expectedMsgId = RANGE;
        transmitPollAck();
        noteActivity();
      }
      else if (msgId == RANGE) {
        timeRangeReceived = DW1000Ng::getReceiveTimestamp();
        expectedMsgId = POLL;
        if (!protocolFailed) {
          timePollSent = DW1000NgUtils::bytesAsValue(r_data + 1, LENGTH_TIMESTAMP);
          timePollAckReceived = DW1000NgUtils::bytesAsValue(r_data + 6, LENGTH_TIMESTAMP);
          timeRangeSent = DW1000NgUtils::bytesAsValue(r_data + 11, LENGTH_TIMESTAMP);
          // (re-)compute range as two-way ranging is done
          double distance = DW1000NgRanging::computeRangeAsymmetric(timePollSent,
                            timePollReceived,
                            timePollAckSent,
                            timePollAckReceived,
                            timeRangeSent,
                            timeRangeReceived);
          /* Apply simple bias correction */
          distance = DW1000NgRanging::correctRange(distance);

          /*
            String rangeString = "Range: "; rangeString += distance; rangeString += " m";
            rangeString += "\t RX power: "; rangeString += DW1000Ng::getReceivePower(); rangeString += " dBm";
            rangeString += "\t Sampling: "; rangeString += samplingRate; rangeString += " Hz";

            Serial.println(rangeString);
          */
          // Serial.print("distance * DISTANCE_OF_RADIO_INV = ");
          // Serial.println(distance * DISTANCE_OF_RADIO_INV);
          //Serial.print("FP power is [dBm]: "); Serial.print(DW1000Ng::getFirstPathPower());
          //Serial.print("RX power is [dBm]: "); Serial.println(DW1000Ng::getReceivePower());
          //Serial.print("Receive quality: "); Serial.println(DW1000Ng::getReceiveQuality());
          // update sampling rate (each second)
          transmitRangeReport(distance * DISTANCE_OF_RADIO_INV); // DISTANCE_OF_RADIO_INV = arround200
          successRangingCount++;
          if (curMillis - rangingCountPeriod > 1000) {
            samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
            rangingCountPeriod = curMillis;
            successRangingCount = 0;
          }
        }
        else {
          transmitRangeFailed();
        }

        noteActivity();
      }
      /////////////// exchange finish
    }

    ///////////////
  }
}
void blinkLED() {
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}
