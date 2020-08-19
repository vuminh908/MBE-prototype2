// Simple serial reader to debug serial communication
// Prints anything received in serial port to USB serial port in readable form

#define SERIAL     Serial1  // Serial port to use for input
#define WRITE_HDR  0x96
#define RETURN_HDR 0x69

const unsigned long baudRate = 115200;
byte rb;
char out[3]; // 2 to represent byte value in hex, 1 more for null terminator
bool printCRLF = false;

bool sameLine = false; // <-- Set to true to print bursts of bytes on same line

void setup() {
  Serial.begin(baudRate);
  SERIAL.begin(baudRate);
  
  // Turn on built-in LED so we know the Teensy is on and the setup completed
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Display message
  Serial.println("=== Serial RX ===");
} // End setup function

void loop() {
  while (SERIAL.available() > 0)
  {
    rb = SERIAL.read();

    if (rb == WRITE_HDR || rb == RETURN_HDR)
      Serial.println();

    sprintf(out, "%02X ", rb);
    Serial.write(out);

    if (sameLine)
      printCRLF = true;
    else
      Serial.println();
  }

  if (printCRLF && sameLine)
  {
    Serial.println();
    printCRLF = false;
  }
} // End loop function
