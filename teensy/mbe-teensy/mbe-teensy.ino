// Serial port read:
//    Code from user Robin2 on Arudino Forum used as reference
//    https://forum.arduino.cc/index.php?topic=288234.0
//
// Interfacing with Teensy ADC:
//    Examples by pedvide, author of Teensy ADC library, used as reference
//    http://pedvide.github.io/ADC/docs/Teensy_3_6_html/index.html

#include <ADC.h>

#define WRITE_HEADER        0x96
#define RETURN_HEADER       0x69
#define REG_POSITION_NEW    0x1E
#define REG_POSITION        0x0C
#define SERIAL_TX           Serial1
#define SERIAL_RX           Serial2

// Number of links (at least 1, support for up to 5)
byte numLinks = 1; // 1 by default
const byte maxNumLinks = 5;

// Current (torque) and position values received for each link
uint16_t rawTorq1;
uint16_t rawTorq2;
uint16_t rawTorq3;
uint16_t rawTorq4;
uint16_t rawTorq5;

uint16_t rawPos1;
uint16_t rawPos2;
uint16_t rawPos3;
uint16_t rawPos4;
uint16_t rawPos5;

// ADC variables
ADC *adc = new ADC();
const byte torqPins[maxNumLinks] = {A0, A1, A2, A3, A4};

// Values for parsing input from or writing output to serial port
// 3 digits, a decimal point, 2 decimal places, and a null terminator ('\0')
const unsigned long baudRate = 115200;
const byte numCharsIn = 7;
char input1[numCharsIn];
char input2[numCharsIn];
char input3[numCharsIn];
char input4[numCharsIn];
char input5[numCharsIn];
boolean newData  = false;
const char startMarker1 = 'a';
const char startMarker2 = 'b';
const char startMarker3 = 'c';
const char startMarker4 = 'd';
const char startMarker5 = 'e';
const char endMarkerIn  = '\r';
const char startMarkerN = 'n';

// (6 chars * 10 values) + 1 for endmarker + 1 for null terminator
const byte numCharsOut = 62;
const char startMarkerOut1 = 'A';
const char startMarkerOut2 = 'B';
const char startMarkerOut3 = 'C';
const char startMarkerOut4 = 'D';
const char startMarkerOut5 = 'E';
const char endMarkerOut = '\r';
String outputStr;
char output[numCharsOut];

// Values for packets to/from servos
const byte maxPktLen = 7;
byte outputPkts[maxPktLen];
const byte packetDataLength = 2;
const byte minPacketLength = 5;

// Angle values for sending to links
// Will be 100x the actual values to avoid truncating the first 2 decimals
uint16_t angle1;
uint16_t angle2;
uint16_t angle3;
uint16_t angle4;
uint16_t angle5;

void setup()
{
  Serial.begin(baudRate);

  SERIAL_TX.begin(baudRate);
  SERIAL_RX.begin(baudRate);

  // ADC
  for (int i = 0; i < maxNumLinks; i++)
    pinMode(torqPins[i], INPUT);

  adc->adc0->setAveraging(8);
  adc->adc0->setResolution(16);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED_16BITS);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);

  // Turn on built-in LED so we know the Teensy is on and the setup completed
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
} // End setup function


void loop()
{
  recieveCommand();

  if(newData)
  {
    //readServoData();
    /*
    Serial.print(rawPos1);
    Serial.print('\t');
    Serial.print(rawPos2);
    Serial.print('\t');
    Serial.print(rawPos3);
    Serial.print('\t');
    Serial.print(rawPos4);
    Serial.print('\t');
    Serial.print(rawPos5);
    Serial.print("\t\t");
    /**/
    /*
    Serial.print(rawTorq1);
    Serial.print('\t');
    Serial.print(rawTorq2);
    Serial.print('\t');
    Serial.print(rawTorq3);
    Serial.print('\t');
    Serial.print(rawTorq4);
    Serial.print('\t');
    Serial.println(rawTorq5);
    /**/
    
    //reportBack();
    /*Serial.println();*/

    sendAngleData();
    /**/
    Serial.print(angle1);
    Serial.print("\t");
    Serial.print(angle2);
    Serial.print("\t");
    Serial.print(angle3);
    Serial.print("\t");
    Serial.print(angle4);
    Serial.print("\t");
    Serial.println(angle5);
    /**/
    
    newData = false;
  }

} // End loop function


// Receive servo angles from serial port (can input through serial monitor or LabVIEW)
// Reads value up to four significant digits (with decimal point)
void recieveCommand()
{
  static byte ndx1 = 0;
  static byte ndx2 = 0;
  static byte ndx3 = 0;
  static byte ndx4 = 0;
  static byte ndx5 = 0;
  static enum {NOREAD, READ1, READ2, READ3, READ4, READ5, READN} readState = NOREAD;

  static char rc;

  while (Serial.available() > 0 && newData == false)
  {
    // Read next character
    rc = Serial.read();

    // State machine for parsing input string from serial port
    if (readState != NOREAD)
    {

      switch (readState)
      {
        case READN: // Read new link number (should be a single value)
          {
            if(rc != endMarkerIn)
            {
              // rc should be a character representing a value between 1 and maxNumLinks in ASCII
              byte tempNum = rc - 0x30;
              if(tempNum >= 1 && tempNum <= maxNumLinks)
              {
                numLinks = tempNum;
              }
              else // Erroneous link number, don't modify numLinks, drop attempted read and move on
              {
                readState = NOREAD;
              }
            }
            else // rc is endMarkerIn, finish reading
            {
              readState = NOREAD;
            }
          }
        case READ1: // Read angle for servo 1
          {
            if (rc != startMarker2)
            {
              input1[ndx1] = rc;
              ndx1++;
              if (ndx1 >= numCharsIn - 1)
              {
                // More characters than expected likely due to having missed next start marker
                // Abort current read and don't set newData to true
                ndx1 = 0;
                readState = NOREAD;
                //ndx1 = numCharsIn - 1;
              }
            }
            else // rc is startMarker2
            {
              input1[ndx1] = '\0';
              ndx1 = 0;
              if(numLinks >= 2)
              {
                // Start reading servo 2 angle
                readState = READ2;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }
        case READ2: // Read angle for servo 2
          {
            if (rc != startMarker3)
            {
              input2[ndx2] = rc;
              ndx2++;
              if (ndx2 >= numCharsIn - 1)
              {
                // More characters than expected likely due to having missed next start marker
                // Abort current read and don't set newData to true
                ndx2 = 0;
                readState = NOREAD;
                //ndx2 = numCharsIn - 1;
              }
            }
            else // rc is startMarker3
            {
              input2[ndx2] = '\0';
              ndx2 = 0;
              if(numLinks >= 3)
              {
                // Start reading servo 3 angle
                readState = READ3;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }
        case READ3: // Read angle for servo 3
          {
            if (rc != startMarker4)
            {
              input3[ndx3] = rc;
              ndx3++;
              if (ndx3 >= numCharsIn - 1)
              {
                // More characters than expected likely due to having missed next start marker
                // Abort current read and don't set newData to true
                ndx3 = 0;
                readState = NOREAD;
                //ndx3 = numCharsIn - 1;
              }
            }
            else // rc is startMarker4
            {
              input3[ndx3] = '\0';
              ndx3 = 0;
              if(numLinks >= 4)
              {
                // Start reading servo 4 angle
                readState = READ4;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }
        case READ4: // Read angle for servo 4
          {
            if (rc != startMarker5)
            {
              input4[ndx4] = rc;
              ndx4++;
              if (ndx4 >= numCharsIn - 1)
              {
                // More characters than expected likely due to having missed next start marker
                // Abort current read and don't set newData to true
                ndx4 = 0;
                readState = NOREAD;
                //ndx4 = numCharsIn - 1;
              }
            }
            else // rc is startMarker5
            {
              input4[ndx4] = '\0';
              ndx4 = 0;
              if(numLinks >= 5)
              {
                // Start reading servo 5 angle
                readState = READ5;
              }
              else
              {
                readState = NOREAD;
                newData = true;
              }
            }
            break;
          }
        case READ5: // Read angle for servo 5
          {
            if (rc != endMarkerIn)
            {
              input5[ndx5] = rc;
              ndx5++;
              if (ndx5 >= numCharsIn - 1)
              {
                // More characters than expected likely due to having missed next start marker
                // Abort current read and don't set newData to true
                ndx5 = 0;
                readState = NOREAD;
                //ndx5 = numCharsIn - 1;
              }
            }
            else // rc is endMarkerIn, finish reading
            {
              input5[ndx5] = '\0';
              ndx5 = 0;
              readState = NOREAD;
              newData = true;
            }
            break;
          }
        default: // Should not reach here
          {
            // Just in case, reset everything
            readState = NOREAD;
            ndx1 = 0;
            ndx2 = 0;
            ndx3 = 0;
            ndx4 = 0;
            ndx5 = 0;
            break;
          }
      }
    }
    else if(rc == startMarkerN)  // Begin reading link number config string
    {
      readState = READN;
    }
    else if (rc == startMarker1) // Begin reading servo angles string
    {
      readState = READ1;
    }

  } // End while loop

} // End recieveCommand function


// Send angles to each link servo
void sendAngleData()
{
  uint16_t mappedAngle1;
  uint16_t mappedAngle2;
  uint16_t mappedAngle3;
  uint16_t mappedAngle4;
  uint16_t mappedAngle5;

  // Can safely assume at least 1 link
  angle1 = atof(input1) * 100;
  angle1 = constrain(angle1, 0, 18000);
  mappedAngle1 = map(angle1, 0, 18000, 400, 5600);
  sendAnglePacket(1, mappedAngle1);

  if(numLinks >= 2)
  {
    angle2 = atof(input2) * 100;
    angle2 = constrain(angle2, 0, 18000);
    mappedAngle2 = map(angle2, 0, 18000, 400, 5600);
    sendAnglePacket(2, mappedAngle2);
  }

  if(numLinks >= 3)
  {
    angle3 = atof(input3) * 100;
    angle3 = constrain(angle3, 0, 18000);
    mappedAngle3 = map(angle3, 0, 18000, 400, 5600);
    sendAnglePacket(3, mappedAngle3);
  }

  if(numLinks >= 4)
  {
    angle4 = atof(input4) * 100;
    angle4 = constrain(angle4, 0, 18000);
    mappedAngle4 = map(angle4, 0, 18000, 400, 5600);
    sendAnglePacket(4, mappedAngle4);
  }

  if(numLinks >= 5)
  {
    angle5 = atof(input5) * 100;
    angle5 = constrain(angle5, 0, 18000);
    mappedAngle5 = map(angle5, 0, 18000, 400, 5600);
    sendAnglePacket(5, mappedAngle5);
  }
  
} // End sendAngleData function


// Read raw servo data (position and torque) from links
// TODO decouple readPositionData from requestPositionData
void readServoData()
{
  // Can safely assume at least 1 link
  rawPos1 = requestPositionData(1);
  rawTorq1 = readTorqueData(1);

  if(numLinks >= 2)
  {
    rawPos2 = requestPositionData(2);
    rawTorq2 = readTorqueData(2);
  }
  else
  {
    rawPos2 = 0;
    rawTorq2 = 0;
  }

  if(numLinks >= 3)
  {
    rawPos3 = requestPositionData(3);
    rawTorq3 = readTorqueData(3);
  }
  else
  {
    rawPos3 = 0;
    rawTorq3 = 0;
  }

  if(numLinks >= 4)
  {
    rawPos4 = requestPositionData(4);
    rawTorq4 = readTorqueData(4);
  }
  else
  {
    rawPos4 = 0;
    rawTorq4 = 0;
  }

  if(numLinks >= 5)
  {
    rawPos5 = requestPositionData(5);
    rawTorq5 = readTorqueData(5);
  }
  else
  {
    rawPos5 = 0;
    rawTorq5 = 0;
  }
} // End readServoData function


// Write servo values back to serial port (can be read in through LabVIEW or serial monitor)
void reportBack()
{
  // Position values with corresponding start markers,
  // then torque values with corresponding start markers (startMarkerOut)
  // and finally the end marker
  outputStr = startMarker1 + String(rawPos1) + startMarkerOut1 + String(rawTorq1) +
              startMarker2 + String(rawPos2) + startMarkerOut2 + String(rawTorq2) +
              startMarker3 + String(rawPos3) + startMarkerOut3 + String(rawTorq3) +
              startMarker4 + String(rawPos4) + startMarkerOut4 + String(rawTorq4) +
              startMarker5 + String(rawPos5) + startMarkerOut5 + String(rawTorq5) +
              endMarkerOut;

  outputStr.toCharArray(output, numCharsOut);

  Serial.write(output);
  //Serial.println(outputStr);
}


// Send packet to specified servo to set angle
void sendAnglePacket(byte servoId, uint16_t mappedAngle)
{
  outputPkts[0] = WRITE_HEADER;
  outputPkts[1] = servoId;
  outputPkts[2] = REG_POSITION_NEW;
  outputPkts[3] = packetDataLength; // Reg data length
  outputPkts[4] = (byte)(mappedAngle & 0xFF); // Data lower byte
  outputPkts[5] = (byte)((mappedAngle & 0xFF00) >> 8); // Data higher byte
  outputPkts[6] = computePktChecksum(packetDataLength);

  SERIAL_TX.write(outputPkts, (minPacketLength + packetDataLength));
} // End sendAngleData function


// Send packet to servo to receive position packet back (also calls read function)
uint16_t requestPositionData(byte servoId)
{
  uint16_t posData = 0;

  outputPkts[0] = WRITE_HEADER;
  outputPkts[1] = servoId;
  outputPkts[2] = REG_POSITION;
  outputPkts[3] = 0; // Reg data length
  outputPkts[4] = computePktChecksum(0);

  SERIAL_TX.write(outputPkts, minPacketLength);

  // Temporarily disable Serial1 to set pin 1 low to allow SBUS high
  // Re-enable after reading returned data from servo
  SERIAL_TX.end();
  SERIAL_RX.clear();
  digitalWrite(1, LOW);
  delay(20);  // Typ. 20 ms delay before servo returns data
  posData = readPositionData(servoId);
  SERIAL_TX.begin(baudRate);
  return posData;
} // End requestPositionData function


// Read packet returned from servo via Serial2
uint16_t readPositionData(byte servoId)
{
  static byte rb;
  static enum {HEADER, ID, ADDR, LEN, DATA_L, DATA_H, CHKSM} readPosState = HEADER;

  byte recId, recAddr, recLen, dataL, dataH;
  unsigned int checksum;
  bool validRX = true;
  uint16_t rawPos = 0;

  // Serial.print("  ");
  while (SERIAL_RX.available())
  {
    rb = SERIAL_RX.read();
    // if(rb < 0x10)
    //   Serial.print('0');
    // Serial.print(rb, HEX);
    // Serial.print(' ');

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
          if (servoId != 0 && recId != servoId)
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
          rawPos = (short)(dataH) << 8;
          rawPos += dataL;
          // Serial.println("\n  Position: " + String(rawPos)); // Test print
          goto readPosRet;
      }
    }
  }

readPosRet:
  SERIAL_RX.clear();
  readPosState = HEADER;
  return rawPos;
} // End readPositionData function


// Read voltage from current sensor (for torque) via ADC
uint16_t readTorqueData(byte servoId)
{
  uint16_t rawTorq = 0;
  byte p = constrain(servoId, 1, numLinks);

  rawTorq = adc->adc0->analogRead(torqPins[p]);
  rawTorq = rawTorq & 0xFFF8; // Filter out some small noise (use 13 out of 16 bits)
  // Serial.println("  Torque: " + String(rawTorq)); // Test print
  return rawTorq;
}


// Compute checksum of given length of outputPkts
byte computePktChecksum(byte dataLength)
{
  unsigned short checksum = 0;
  for (byte j = 1; j <= (minPacketLength - 2 + dataLength); j++)
    checksum += outputPkts[j];

  return (byte)(checksum % 256);
} // End computePktChecksum
