/* ============================================================================
   PROJECT HORIZON  -  PILOT CONTROLLER  (TRANSMITTER)
   ----------------------------------------------------------------------------
   Author : Zyad Hossameldien
   Course : TEJ4M Computer Engineering - Capstone 2026
   School : Earl of March Secondary School
   ----------------------------------------------------------------------------
   Custom hand-built FPV quadcopter ground transmitter.
   Reads the two analog joysticks (throttle, yaw, roll, pitch) and two
   arm/mode switches, packs them into a compact 7-byte radio packet with an
   XOR integrity checksum, and broadcasts it over an NRF24L01 (PA+LNA) link
   to the drone's onboard receiver.

   Radio link settings must match the receiver exactly:
     - Pipe address : 0xABCDABCD71
     - Channel      : 100
     - Data rate    : 250KBPS
     - PA level     : MAX
     - CRC          : 16-bit
     - AutoAck      : OFF (one-way broadcast link)
   ============================================================================ */

#include <SPI.h>          // SPI bus - used by the NRF24 radio module
#include <nRF24L01.h>     // Low-level NRF24L01 register definitions
#include <RF24.h>         // High-level RF24 radio driver library

// Writing pipe address - MUST be identical on the receiver's reading pipe
const uint64_t pipeOut = 0xABCDABCD71LL;
RF24 radio(9, 10);        // RF24 radio: CE pin = D9, CSN pin = D10

// ---- Radio packet layout ----
// All six control values are packed as single bytes (0-255) to keep the
// packet tiny and fast. The final byte is an integrity checksum.
struct Signal {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
  byte aux1;
  byte aux2;
  byte checksum;   // packet integrity check
};

Signal data;

// ---- Throttle calibration (spring-centered: center = 0, push up = climb) ----
const int THR_CENTER   = 530;    // Raw analog value at the throttle stick's rest/center point
const int THR_TOP      = 1023;   // Raw analog value at the throttle stick's full-up position
const int THR_DEADBAND = 18;     // Ignore tiny movements around center to stop jitter

// Reset all controls to safe neutral values (throttle 0, sticks centered, switches off).
// Called on startup and whenever a safe default is needed.
void ResetData() {
  data.throttle = 0;
  data.pitch    = 127;
  data.roll     = 127;
  data.yaw      = 127;
  data.aux1     = 0;
  data.aux2     = 0;
  data.checksum = 0;
}

// XOR checksum with a constant salt so all-zero or all-rail packets fail
// XORs every control byte together with 0xA5. The receiver recomputes this and
// rejects the packet if it doesn't match - this is what defeated the motor-EMI
// packet corruption that was causing random disarms.
byte calcChecksum(const Signal &s) {
  return s.throttle ^ s.pitch ^ s.roll ^ s.yaw ^ s.aux1 ^ s.aux2 ^ 0xA5;
}

// Maps a raw 0-1023 analog reading to a 0-255 byte using a calibrated CENTER
// point, so the physical mechanical center of the stick maps cleanly to 127.
// Used for yaw, roll, and pitch (sticks that rest in the middle).
// 'reverse' flips the axis direction if a stick is wired/mounted backwards.
int Border_Map_Cal(int val, int lower, int middle, int upper, bool reverse) {
  val = constrain(val, lower, upper);
  int out;
  if (val < middle)
    out = map(val, lower, middle, 0, 127);
  else
    out = map(val, middle, upper, 127, 255);
  out = constrain(out, 0, 255);
  if (reverse) out = 255 - out;
  return out;
}

// Throttle anchored at center: below center -> 0, above -> 0..255 linear
// Throttle-specific mapping: anything at or below center returns 0 (motors idle),
// and travel above center scales linearly up to full throttle (255).
int ThrottleFromCenter(int val, int center, int top) {
  if (val <= center + THR_DEADBAND) return 0;
  int out = map(val, center + THR_DEADBAND, top, 0, 255);
  return constrain(out, 0, 255);
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== PROJECT HORIZON TRANSMITTER ===");

  // The two switch inputs use the internal pull-up resistors, so an open
  // switch reads HIGH and a pressed switch (to GND) reads LOW.
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  ResetData();

  // Bring up the radio; halt here if the module isn't detected on SPI.
  if (!radio.begin()) {
    Serial.println("RADIO FAILED");
    while (1);
  }

  // ---- Radio configuration (must mirror the receiver) ----
  radio.openWritingPipe(pipeOut);
  radio.setChannel(100);
  radio.setAutoAck(false);            // ENABLE hardware CRC + retransmit           // 3*250us delay, up to 5 retries
  radio.setDataRate(RF24_250KBPS);    // Slowest rate = best range/reliability
  radio.setPALevel(RF24_PA_MAX);      // Max transmit power (PA+LNA module)
  radio.setCRCLength(RF24_CRC_16);   // strongest CRC
  radio.stopListening();              // Transmitter only - never receives

  Serial.println("OK — A0=Throttle(center=0,up=climb) A1=Yaw A2=Roll A3=Pitch");
}

unsigned long lastPrint = 0;
unsigned long packetCount = 0;

void loop() {
  // ---- 1) Read all four analog stick axes ----
  int rawYaw      = analogRead(A1);
  int rawThrottle = analogRead(A0);
  int rawRoll     = analogRead(A2);
  int rawPitch    = analogRead(A3);

  // ---- 2) Convert raw readings into calibrated 0-255 control bytes ----
  data.yaw      = Border_Map_Cal(rawYaw,   0, 513, 1023, false);
  data.throttle = ThrottleFromCenter(rawThrottle, THR_CENTER, THR_TOP);
  data.pitch    = Border_Map_Cal(rawPitch, 0, 521, 1023, false);
  data.roll     = Border_Map_Cal(rawRoll,  0, 516, 1023, false);

  // ---- 3) Read the two switches (inverted because of INPUT_PULLUP) ----
  data.aux1 = !digitalRead(2);
  data.aux2 = !digitalRead(3);

  data.checksum = calcChecksum(data);   // compute AFTER all fields set

  // ---- 4) Broadcast the packet over the radio ----
  radio.write(&data, sizeof(Signal));
  packetCount++;

  // ---- 5) Print a status report twice a second for debugging ----
  if (millis() - lastPrint > 500) {
    Serial.println("--- TX STATUS ---");
    Serial.print("Packets: "); Serial.println(packetCount);
    Serial.print("THROTTLE:"); Serial.print(data.throttle);
    Serial.print(" PITCH:");   Serial.print(data.pitch);
    Serial.print(" ROLL:");    Serial.print(data.roll);
    Serial.print(" YAW:");     Serial.println(data.yaw);
    Serial.print("RAW A0(thr):"); Serial.print(rawThrottle);
    Serial.print(" A1(yaw):");     Serial.print(rawYaw);
    Serial.print(" A2(roll):");    Serial.print(rawRoll);
    Serial.print(" A3(pitch):");   Serial.println(rawPitch);
    Serial.print("AUX1(D2): "); Serial.print(data.aux1);
    Serial.print("  AUX2(D3): "); Serial.println(data.aux2);
    Serial.print("CHK: "); Serial.println(data.checksum);
    lastPrint = millis();
  }
}
