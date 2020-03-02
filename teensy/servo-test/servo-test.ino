// Serial port read:
//    Code from user Robin2 on Arudino Forum used as reference
//    https://forum.arduino.cc/index.php?topic=288234.0
//
// Target Board: Teensy 3.2
// For use with Hitec D-series servos (i.e. D956WP)
//
// Input 'a' followed by the desired servo angle into the Serial monitor (use carriage return \r)
// Input '~' to toggle demo sine wave (about 0.625 Hz frequency)
// Input only carriage return \r to read ID register
// Input '!' followed by desired ID and \r to set ID of servo (to keep ID set, power-cycle servo)
// Input '*' to perform factory reset
// Input '?' to set ID field in packets to 0 (broadcast)
// Input 'q', 'w', or 'e' to set ID field in packets to 1, 2, or 3, respectively
// Input '.' to reset ID field in packets to what it was before

//#include <Servo.h>
//#include <ADC.h>

#define WRITE_HEADER        0x96
#define RETURN_HEADER       0x69
#define REG_POSITION_NEW    0x1E
#define REG_POSITION        0xC
#define REG_ID              0x32
#define REG_CONFIG_SAVE     0x70
#define REG_FACTORY_DEFAULT 0x6E
#define SERIAL_TX           Serial1
#define SERIAL_RX           Serial2

// Servo servo;
// const byte servoPin = 9;

// ADC *adc = new ADC();
// const byte torqPin = A2;
// const byte posPin = A9;
// uint16_t rawTorq; // Raw ADC value of current sensor output (proportional to torque)
// uint16_t rawPos;  // Raw ADC value of feedback potentiometer voltage

const unsigned long baudRate = 115200;
const unsigned long timeDelay = 100; // Delay, in milliseconds, between changing angle values
unsigned long timeStamp;
boolean demo = false;
boolean dir = true; // Indicates if counting up (true) or down (false) with angleIndex
const byte numAngles = 9;
const unsigned short angleArr[numAngles] = {0, 7, 26, 56, 90, 124, 154, 173, 180};
byte angleIndex = 0; // Index in angleArr

unsigned short angle = 90; // Angle value received from serial input
const byte numCharsIn = 4;
char inputArr[numCharsIn];
boolean newData  = false;
const char startMarkerIn = 'a';
const char endMarkerIn = '\r';
const char demoMarker = '~';
const char setIdMarker = '!';
const char resetIdMarker = '?';
const char restoreIdMarker = '.';
const char factoryRstMarker = '*';

//const char startMarkerOut = 'a';
//const char endMarkerOut = '!';
String outputStr;
const byte numCharsOut = 7;
byte outputArr[numCharsOut];

byte servoId = 0; // 0 or 255 for broadcast
byte expectedId = 0; // What ID is thought to have been set to in the servo
const byte minPacketLength = 5;
unsigned short angleData = 3000; // Values range from 400 to 5600 by default (3000 <-> 90 degrees)
const byte packetDataLength = 2;
unsigned int checksum = 0;


void setup()
{
  Serial.begin(baudRate);
  //Serial.begin(9600);

  pinMode(1, OUTPUT);
  SERIAL_TX.begin(baudRate);
  SERIAL_RX.begin(baudRate);

  // pinMode(torqPin, INPUT);
  // pinMode(posPin, INPUT);
  // adc->setAveraging(16);
  // adc->setResolution(16);
  // adc->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED);
  // adc->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);
  
  // servo.attach(servoPin);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // timeStamp = micros();
} // End setup function

void loop()
{
  receiveCommand();

  if(newData)
  {
    angle = (!demo) ? atoi(inputArr) : angle;
    angle = constrain(angle, 0, 180);
    angleData = map(angle, 0, 180, 400, 5600);
    newData = false;

    // Write angle to servo
    sendAngleData();
    requestPositionData();
  }


  // Demo sine wave
  if((millis() - timeStamp) >= timeDelay)
  {
    if (demo)
    {
      angle = angleArr[angleIndex];

      if (angle == 180 || angle == 0)
        dir = !dir;

      if (dir)
        angleIndex++;
      else
        angleIndex--;

      newData = true;
    }
    //rawTorq = adc->analogRead(torqPin);
    //rawPos = adc->analogRead(posPin);
    
    //outputStr = "Torque: " + String(rawTorq) + "\tPosition: " + String(rawPos);
    //outputStr.toCharArray(outputArr, numCharsOut);
    //Serial.write(outputArr);
    //Serial.println(outputStr);

    timeStamp = millis();
  }

} // End loop function


// Receive incoming packet from computer via USB serial
void receiveCommand()
{
  static byte ndx = 0;
  static enum {NOREAD, READ, SETID} readState = NOREAD;

  static char rc;

  while (Serial.available() > 0 && newData == false)
  {
    rc = Serial.read();

    if (readState != NOREAD)
    {
      if (rc != endMarkerIn)
      {
        inputArr[ndx] = rc;
        ndx++;
        if (ndx >= numCharsIn)
        {
          ndx = numCharsIn - 1;
        }
      }
      else // rc is endMarker, finish reading
      {
        inputArr[ndx] = '\0';
        ndx = 0;

        if (readState == SETID)
        {
          byte newId = (byte)(atoi(inputArr));
          setId(newId);
        }
        else
        {
          newData = true;
        }
        readState = NOREAD;
      }
    }
    else if (!demo && rc != demoMarker)
    {
      switch (rc)
      {
        case startMarkerIn: // Begin reading angle
          readState = READ;
          break;
        case setIdMarker: // Begin reading ID to set
          readState = SETID;
          break;
        case endMarkerIn: // Request/Read servo ID
          requestId();
          break;
        case resetIdMarker: // Reset ID (to 0) of packets Teensy sends
          Serial.clear();
          servoId = 0;
          Serial.println("ID of packets reset to 0");
          break;
        case 'q': // Set ID in packets to 1
          Serial.clear();
          servoId = 1;
          Serial.println("ID of packets set to 1");
          break;
        case 'w': // Set ID in packets to 2
          Serial.clear();
          servoId = 2;
          Serial.println("ID of packets set to 2");
          break;
        case 'e': // Set ID in packets to 3
          Serial.clear();
          servoId = 3;
          break;
        case restoreIdMarker: // Set ID of packets Teensy sends to what it was before
          Serial.clear();
          servoId = expectedId;
          Serial.println("ID of packets restored to " + String(expectedId));
          break;
        case factoryRstMarker: // Perform factory reset
          Serial.clear();
          doFactoryReset();
          break;
      }
    }
    else if (rc == demoMarker) // Toggle demo
    {
      Serial.clear();
      demo = !demo;
      if (demo)
      {
        // Start at 0 degrees when enabling demo
        angleIndex = 0;
        angle = angleArr[angleIndex];
        angleIndex++;
        dir = true;
      }
      else
      {
        // Default to 90 degrees when disabling demo
        inputArr[0] = '9';
        inputArr[1] = '0';
        inputArr[2] = '\0';
      }
      newData = true;
      Serial.clear();
    }

  } // End while loop

} // End receiveCommand function


// Send packet to servo to set angle
void sendAngleData()
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = servoId;
  outputArr[2] = REG_POSITION_NEW;
  outputArr[3] = packetDataLength; // Reg data length
  outputArr[4] = (byte)(angleData & 0xFF); // Data lower byte
  outputArr[5] = (byte)((angleData & 0xFF00) >> 8); // Data higher byte
  outputArr[6] = computeChecksum(packetDataLength);

  Serial.println("Angle: " + String(angle) + "\tValue: " + String(angleData));
  printPacketToSend(packetDataLength);
  SERIAL_TX.write(outputArr, (minPacketLength + packetDataLength));
} // End sendAngleData function


// Send packet to servo to receive position packet back
void requestPositionData()
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = servoId;
  outputArr[2] = REG_POSITION;
  outputArr[3] = 0; // Reg data length
  outputArr[4] = computeChecksum(0);

  printPacketToSend(0);
  SERIAL_TX.write(outputArr, minPacketLength);

  // Temporarily disable Serial1 to set pin 1 low to allow SBUS high
  // Re-enable after reading returned data from servo
  SERIAL_TX.end();
  SERIAL_RX.clear();
  digitalWrite(1, LOW);
  delay(20);  // Typ. 20 ms delay before servo returns data
  readPositionData(servoId);
  SERIAL_TX.begin(baudRate);
} // End requestPositionData function


// Read packet returned from servo via Serial2
bool readPositionData(byte id)
{
  static byte rb;
  static enum {HEADER, ID, ADDR, LEN, DATA_L, DATA_H, CHKSM} readPosState = HEADER;

  byte recId, recAddr, recLen, dataL, dataH;
  short position;
  bool validRX = true;
  bool retVal = false;

  Serial.print("  ");
  while (SERIAL_RX.available())
  {
    rb = SERIAL_RX.read();
    if(rb < 0x10)
      Serial.print('0');
    Serial.print(rb, HEX);
    Serial.print(' ');

    if (validRX)
    {
      switch (readPosState)
      {
        case HEADER:
          if (rb == RETURN_HEADER)
            readPosState = ID;
          break;
        case ID:
          recId = rb;
          if (recId != id)
            validRX = false;
          readPosState = ADDR;
          break;
        case ADDR:
          recAddr = rb;
          if (recAddr != REG_POSITION)
            validRX = false;
          readPosState = LEN;
          break;
        case LEN:
          recLen = rb;
          if (recLen != packetDataLength)
            validRX = false;
          readPosState = DATA_L;
          break;
        case DATA_L:
          dataL = rb;
          readPosState = DATA_H;
          break;
        case DATA_H:
          dataH = rb;
          readPosState = CHKSM;
          break;
        case CHKSM:
          checksum = recId + recAddr + recLen + dataL + dataH;
          if ((byte)(checksum % 256) != rb)
            validRX = false;
          position = (short)(dataH) << 8;
          position += dataL;
          Serial.println("\n  Position: " + String(position));
          retVal = true;
          goto readPosRet;
      }
    }
  }
  Serial.println();
  // TODO store received data and pass on to PC

readPosRet:
  SERIAL_RX.clear();
  readPosState = HEADER;
  return retVal;
} // End readPositionData function


// Request read of servo's ID register, then read what is returned
void requestId()
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = 0;
  outputArr[2] = REG_ID;
  outputArr[3] = 0; // Reg data length
  outputArr[4] = computeChecksum(0);

  Serial.println("Request/Read ID:");
  printPacketToSend(0);
  SERIAL_TX.write(outputArr, minPacketLength);

  // Temporarily disable Serial1 to set pin 1 low to allow SBUS high
  // Re-enable after reading returned data from servo
  SERIAL_TX.end();
  SERIAL_RX.clear();
  digitalWrite(1, LOW);
  delay(20);  // Typ. 20 ms delay before servo returns data
  readId();
  SERIAL_TX.begin(baudRate);
} // End requestId function


// Read ID value returned from servo
void readId()
{
  static byte rb;

  Serial.print("  ");
  while (SERIAL_RX.available())
  {
    rb = SERIAL_RX.read();
    if(rb < 0x10)
      Serial.print('0');
    Serial.print(rb, HEX);
    Serial.print(' ');
  }
  Serial.println();
} // End readId function


// Set ID value for servo, then save config
void setId(byte id)
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = servoId;
  outputArr[2] = REG_ID;
  outputArr[3] = packetDataLength; // Reg data length
  outputArr[4] = id;
  outputArr[5] = 0x00;
  outputArr[6] = computeChecksum(packetDataLength);

  Serial.println("Set ID to: " + String(id));
  printPacketToSend(packetDataLength);
  SERIAL_TX.write(outputArr, (minPacketLength + packetDataLength));
  saveConfig(); // Save
  Serial.println("To keep ID, power cycle servo");
  expectedId = id;
  servoId = expectedId;
} // End setId


// Perform factory reset
void doFactoryReset()
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = servoId;
  outputArr[2] = REG_FACTORY_DEFAULT;
  outputArr[3] = 0; // Reg data length
  outputArr[4] = computeChecksum(0);

  Serial.println("FACTORY RESET:");
  printPacketToSend(0);
  SERIAL_TX.write(outputArr, minPacketLength);
} // End doFactoryReset


// Send save command
void saveConfig()
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = servoId;
  outputArr[2] = REG_CONFIG_SAVE;
  outputArr[3] = packetDataLength; // Reg data length
  outputArr[4] = 0xFF;
  outputArr[5] = 0xFF;
  outputArr[6] = computeChecksum(2);

  printPacketToSend(2);
  SERIAL_TX.write(outputArr, (minPacketLength + packetDataLength));
} // End saveConfig


// Compute checksum of given length of outputArr
byte computeChecksum(byte dataLength)
{
  checksum = 0;
  for (int j = 1; j <= (minPacketLength - 2 + dataLength); j++)
    checksum += outputArr[j];

  return (byte)(checksum % 256);
} // End computeChecksum


void printPacketToSend(byte dataLength)
{
  Serial.print("  ");
  for(int i = 0; i < (minPacketLength + dataLength); i++)
  {
    if(outputArr[i] < 0x10)
      Serial.print('0');
    Serial.print(outputArr[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
} // End printPacketToSend
