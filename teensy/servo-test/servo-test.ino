// Serial port read:
//    Code from user Robin2 on Arudino Forum used as reference
//    https://forum.arduino.cc/index.php?topic=288234.0
//
// Input 'a' followed by the desired servo angle into the Serial monitor (use carriage return \r)
// Input only carriage return \r to toggle demo sine wave (about 0.625 Hz frequency)

//#include <Servo.h>
//#include <ADC.h>

#define WRITE_HEADER      0x96
#define RETURN_HEADER     0x69
#define REG_POSITION_NEW  0x1E
#define REG_POSITION      0xC
#define SERIAL_TX         Serial1
#define SERIAL_RX         Serial2

// Servo servo;
// const byte servoPin = 9;

// ADC *adc = new ADC();
// const byte torqPin = A2;
// const byte posPin = A9;
// uint16_t rawTorq; // Raw ADC value of current sensor output (proportional to torque)
// uint16_t rawPos;  // Raw ADC value of feedback potentiometer voltage

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

//const char startMarkerOut = 'a';
//const char endMarkerOut = '!';
String outputStr;
const byte numCharsOut = 7;
byte outputArr[numCharsOut];

byte servoId = 0; // 0 or 255 for broadcast
const byte minPacketLength = 5;
unsigned short angleData = 3000; // Values range from 400 to 5600 by default (3000 <-> 90 degrees)
const byte angleDataLength = 2;
unsigned long checksum = 0;

void setup()
{
  Serial.begin(115200);
  //Serial.begin(9600);

  pinMode(1, OUTPUT);
  SERIAL_TX.begin(115200);
  SERIAL_RX.begin(115200);

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
  recieveAngleData();

  if(newData)
  {
    angle = (!demo) ? atoi(inputArr) : angle;
    angle = constrain(angle, 0, 180);
    angleData = map(angle, 0, 180, 400, 5600);
    outputStr = "Angle: " + String(angle) + "\tValue: " + String(angleData);
    Serial.println(outputStr);
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
void recieveAngleData()
{
  static byte ndx = 0;
  static enum {NOREAD, READ} readState = NOREAD;

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
        readState = NOREAD;
        inputArr[ndx] = '\0';
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarkerIn && !demo) // Begin reading
    {
      readState = READ;
    }
    else if (rc == endMarkerIn) // Toggle demo
    {
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
    }

  } // End while loop

} // End recieveAngleData function


// Send packet to servo to set angle
void sendAngleData()
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = servoId;
  outputArr[2] = REG_POSITION_NEW;
  outputArr[3] = angleDataLength; // Reg data length
  outputArr[4] = (byte)(angleData & 0xFF); // Data lower byte
  outputArr[5] = (byte)((angleData & 0xFF00) >> 8); // Data higher byte
  checksum = servoId + REG_POSITION_NEW + angleDataLength + outputArr[4] + outputArr[5];
  outputArr[6] = (byte)(checksum % 256);

  Serial.print("  ");
  for(int i = 0; i < (minPacketLength + angleDataLength); i++)
  {
    if(outputArr[i] < 0x10)
      Serial.print('0');
    Serial.print(outputArr[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  SERIAL_TX.write(outputArr, (minPacketLength + angleDataLength));
} // End sendAngleData function


// Send packet to servo to receive position packet back
void requestPositionData()
{
  outputArr[0] = WRITE_HEADER;
  outputArr[1] = servoId;
  outputArr[2] = REG_POSITION;
  outputArr[3] = 0; // Reg data length
  checksum = servoId + REG_POSITION;
  outputArr[4] = (byte)(checksum % 256);

  Serial.print("  ");
  for(int i = 0; i < minPacketLength; i++)
  {
    if(outputArr[i] < 0x10)
      Serial.print('0');
    Serial.print(outputArr[i], HEX);
    Serial.print(' ');
  }
  Serial.println();
  SERIAL_TX.write(outputArr, minPacketLength);

  // Temporarily disable Serial1 to set pin 1 low to allow SBUS high
  // Re-enable after reading returned data from servo
  SERIAL_TX.end();
  digitalWrite(1, LOW);
  delay(20);  // Typ. 20 ms delay before servo returns data
  readPositionData();
  SERIAL_TX.begin(115200);
} // End requestPositionData function


// Read packet returned from servo via Serial2
void readPositionData()
{
  byte rb;
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
  // TODO parse received data and pass on to PC
} // End readPositionData function

// TODO helper function for debug printing byte streams to PC?
