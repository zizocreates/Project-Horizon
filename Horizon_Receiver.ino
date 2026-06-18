/* ============================================================================
   PROJECT HORIZON  -  DRONE RECEIVER
   ----------------------------------------------------------------------------
   Author : Zyad Hossameldien
   Course : TEJ4M Computer Engineering - Capstone 2026
   School : Earl of March Secondary School
   ----------------------------------------------------------------------------
   Onboard receiver for the custom hand-built FPV quadcopter.
   Listens for the 7-byte control packets broadcast by the pilot transmitter,
   verifies each packet's XOR checksum, and converts the six control values
   into standard 1000-2000us servo/PWM signals. Those PWM channels feed the
   T1P PWM-to-SBUS converter, which in turn feeds the Betaflight flight
   controller.

   Reliability features:
     - Checksum rejection: corrupt packets (e.g. from motor EMI) are discarded
       and the last-known-good values are held instead of acting on garbage.
     - Tiered timeout: brief signal gaps ride out on held values; only a long
       dropout (>1.2s) triggers a full failsafe that zeroes the throttle.

   Radio settings mirror the transmitter exactly (pipe, channel, rate, PA, CRC,
   AutoAck OFF).
   ============================================================================ */

#include <SPI.h>          // SPI bus - used by the NRF24 radio module
#include <nRF24L01.h>     // Low-level NRF24L01 register definitions
#include <RF24.h>         // High-level RF24 radio driver library
#include <Servo.h>        // Generates the 1000-2000us PWM output channels

// PWM output width (microseconds) for each of the six output channels
int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;

// Six PWM output channels, generated using the Servo library
Servo ch1, ch2, ch3, ch4, ch5, ch6;

// ---- Radio packet layout (MUST match the transmitter byte-for-byte) ----
struct Signal {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
  byte aux1;
  byte aux2;
  byte checksum;
};

Signal data;

// Reading pipe address - MUST match the transmitter's writing pipe
const uint64_t pipeIn = 0xABCDABCD71LL;
RF24 radio(9, 10);        // RF24 radio: CE pin = D9, CSN pin = D10

unsigned long lastRecvTime = 0;    // millis() timestamp of the last good packet
unsigned long packetCount = 0;     // running count of accepted packets
unsigned long rejectCount = 0;     // count of corrupt packets thrown away
bool linked = false;               // true once a valid link is established

// Hold last-good values briefly through noise bursts; full failsafe only after longer loss
const unsigned long HOLD_TIMEOUT_MS     = 500;    // ride out short noise bursts
const unsigned long FAILSAFE_TIMEOUT_MS = 1200;   // declare signal lost after this

// Reset all controls to safe neutral values (throttle 0, sticks centered).
// This is the failsafe state - motors idle if the link is truly lost.
void ResetData() {
  data.throttle = 0;
  data.roll = 127;
  data.pitch = 127;
  data.yaw = 127;
  data.aux1 = 0;
  data.aux2 = 0;
  data.checksum = 0;
}

// Recompute the same XOR checksum the transmitter used, to validate a packet.
byte calcChecksum(const Signal &s) {
  return s.throttle ^ s.pitch ^ s.roll ^ s.yaw ^ s.aux1 ^ s.aux2 ^ 0xA5;
}

// Drain the radio FIFO. For each received packet, recompute the checksum and
// only accept it if it matches; corrupt packets are counted and discarded so
// the drone keeps flying on the last good command instead of a garbled one.
void recvData() {
  while (radio.available()) {
    Signal tmp;
    radio.read(&tmp, sizeof(Signal));
    if (calcChecksum(tmp) == tmp.checksum) {   // ACCEPT only valid packets
      data = tmp;
      lastRecvTime = millis();
      packetCount++;
      if (!linked) {
        linked = true;
        Serial.println("*** LINK ESTABLISHED — RECEIVING DATA ***");
      }
    } else {
      rejectCount++;                            // corrupt -> discard, keep last good
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== PROJECT HORIZON RECEIVER ===");

  // ---- Attach the six PWM output channels to digital pins D2-D7 ----
  ch1.attach(2);  // CH1 roll
  ch2.attach(3);  // CH2 pitch
  ch3.attach(4);  // CH3 throttle
  ch4.attach(5);  // CH4 yaw
  ch5.attach(6);  // CH5 AUX1 arm
  ch6.attach(7);  // CH6 AUX2 flight mode

  ResetData();

  // Bring up the radio; if it fails, loop forever reporting chip status.
  Serial.print("Initializing radio... ");
  if (!radio.begin()) {
    Serial.println("FAILED — check wiring");
    while (1) {
      Serial.print("Chip detected: ");
      Serial.println(radio.isChipConnected() ? "YES" : "NO");
      delay(1000);
    }
  }
  Serial.println("OK");

  // ---- Radio configuration (must mirror the transmitter) ----
  radio.openReadingPipe(1, pipeIn);
  radio.setChannel(100);
  radio.setAutoAck(false);            // MUST match transmitter
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setCRCLength(RF24_CRC_16);
  radio.startListening();             // Receiver mode - always listening

  Serial.println("Waiting for transmitter...");
}

unsigned long lastPrint = 0;

void loop() {
  // ---- 1) Pull in and validate any waiting packets ----
  recvData();

  // ---- 2) How long since the last GOOD packet? ----
  unsigned long sinceLast = millis() - lastRecvTime;

  // ---- 3) Failsafe: if the link has been dead too long, force safe neutral ----
  if (sinceLast > FAILSAFE_TIMEOUT_MS) {
    if (linked) {
      linked = false;
      Serial.println("*** SIGNAL LOST — FAILSAFE (throttle 0) ***");
    }
    ResetData();
  }
  // Between HOLD and FAILSAFE timeouts we keep the last good 'data' (ride out bursts).

  // ---- 4) Convert each control byte (0-255) into a 1000-2000us PWM width ----
  ch_width_1 = map(data.roll,     0, 255, 1000, 2000);
  ch_width_2 = map(data.pitch,    0, 255, 1000, 2000);
  ch_width_3 = map(data.throttle, 0, 255, 1000, 2000);
  ch_width_4 = map(data.yaw,      0, 255, 1000, 2000);
  ch_width_5 = map(data.aux2, 0, 1, 1000, 2000); // CH5 / AUX1 in Betaflight
  ch_width_6 = map(data.aux1, 0, 1, 1000, 2000); // CH6 / AUX2 in Betaflight

  // ---- 5) Write the PWM widths out to the six channels ----
  ch1.writeMicroseconds(ch_width_1);
  ch2.writeMicroseconds(ch_width_2);
  ch3.writeMicroseconds(ch_width_3);
  ch4.writeMicroseconds(ch_width_4);
  ch5.writeMicroseconds(ch_width_5);
  ch6.writeMicroseconds(ch_width_6);

  // ---- 6) Print a status report twice a second for debugging ----
  if (millis() - lastPrint > 500) {
    Serial.println("--- RECEIVER STATUS ---");
    Serial.print("Link: "); Serial.println(linked ? "LINKED" : "NO LINK");
    Serial.print("Good packets: "); Serial.print(packetCount);
    Serial.print("  Rejected (corrupt): "); Serial.println(rejectCount);
    Serial.print("Since last good: "); Serial.print(sinceLast); Serial.println("ms");
    if (linked) {
      Serial.print("THR:"); Serial.print(data.throttle);
      Serial.print(" PITCH:"); Serial.print(data.pitch);
      Serial.print(" ROLL:"); Serial.print(data.roll);
      Serial.print(" YAW:"); Serial.print(data.yaw);
      Serial.print(" AUX1:"); Serial.print(data.aux1);
      Serial.print(" AUX2:"); Serial.println(data.aux2);
      Serial.print("PWM CH3(thr):"); Serial.println(ch_width_3);
    }
    Serial.println("-----------------------");
    lastPrint = millis();
  }
}
