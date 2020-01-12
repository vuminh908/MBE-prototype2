// Serial port read:
//    Code from user Robin2 on Arudino Forum used as reference
//    https://forum.arduino.cc/index.php?topic=288234.0
//
// Input 'a' followed by the desired servo angle into the Serial monitor (use carriage return \r)

//#include <Servo.h>
//#include <ADC.h>

#define WRITE_HEADER 0x96
#define REG_POSITION_NEW 0x1E
#define SERVO_SERIAL Serial1

// Servo servo;
// const byte servoPin = 9;

// ADC *adc = new ADC();
// const byte torqPin = A2;
// const byte posPin = A9;
// uint16_t rawTorq; // Raw ADC value of current sensor output (proportional to torque)
// uint16_t rawPos;  // Raw ADC value of feedback potentiometer voltage

//const unsigned long timeDelay = 100; // Delay, in milliseconds
//unsigned long timeStamp;

unsigned short angle = 90; // Angle value received from serial input
const byte numCharsIn = 4;
char inputArr[numCharsIn];
boolean newData  = false;
const char startMarkerIn = 'a';
const char endMarkerIn = '\r';

//const char startMarkerOut = 'a';
//const char endMarkerOut = '!';
String outputStr;
const byte numCharsOut = 64;
byte outputArr[numCharsOut];

byte servoId = 0; // 0 for broadcast
const byte minPacketLength = 5;
unsigned short angleData = 3000; // Values range from 400 to 5600 by default (3000 <-> 90 degrees)
const byte angleDataLength = 2;
unsigned long checksum = 0;

void setup()
{
  Serial.begin(115200);
  //Serial.begin(9600);

  SERVO_SERIAL.begin(115200);

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
    angle = atoi(inputArr);
    angle = constrain(angle, 0, 180);
    angleData = map(angle, 0, 180, 400, 5600);
    outputStr = "Angle: " + String(angle) + "\tValue: " + String(angleData);
    Serial.println(outputStr);
    newData = false;

    // Write angle to servo
    sendAngleData();
  }

  /*
  // Sample data
  if((millis() - timeStamp) >= timeDelay)
  {
    //rawTorq = adc->analogRead(torqPin);
    //rawPos = adc->analogRead(posPin);
    
    //outputStr = "Torque: " + String(rawTorq) + "\tPosition: " + String(rawPos);
    //outputStr.toCharArray(outputArr, numCharsOut);
    //Serial.write(outputArr);
    //Serial.println(outputStr);

    timeStamp = millis();
  }
  */

} // End loop function


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
    else if (rc == startMarkerIn) // Begin reading
    {
      readState = READ;
    }
  } // End while loop

} // End recieveAngleData function


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
  SERVO_SERIAL.write(outputArr, (minPacketLength + angleDataLength));
} // End sendAngleData function
