#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <VarSpeedServo.h>

VarSpeedServo localServo;
byte rotationSpeed = 100, lastServoAngle = 180, angleOfRotation = 0, Sinusoid = 0, nowAngle = 0, Direction = 0;
bool isNowMoveTo = false, isNowMoveOn = false, isStartMoveTo = false, isStartMoveOn = false;
uint8_t timerfor_moveTo;

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return float((x - in_min) * (out_max - out_min)) / float((in_max - in_min) + out_min);
}

float map(int x, int in_min, int in_max, int out_min, int out_max)
{
  return float((x - in_min) * (out_max - out_min)) / float((in_max - in_min) + out_min);
}

void setSpeed(byte RotationSpeed)
{
  rotationSpeed = RotationSpeed > 0 ? RotationSpeed : rotationSpeed;
}

bool check_control_summ(byte length, byte *array_pointer)
{
  length /= sizeof(byte);
  byte checksum = 0;
  for (byte i = 0; i < length - 1; i++)
  {
    checksum += array_pointer[i];
  }
  return checksum == array_pointer[length - 1];
}

float expRunningAverage(float newVal, float k)
{
  static float filVal = 0;
  filVal += (newVal - filVal) * k;
  return filVal;
}

void receiveEvent(int howMany)
{
  while (Wire.available())
  {
    byte first_input = Wire.read();
    // Serial.print("Case: ");
    // Serial.println(first_input);
    switch (first_input)
    {
    case 0: // moveTo
      byte input_array_0[3];
      for (byte i = 0; i < 3; i++)
      {
        input_array_0[i] = Wire.read();
      }
      if (check_control_summ(sizeof(input_array_0), input_array_0))
      {
        isStartMoveTo = true;
        angleOfRotation = input_array_0[1];
        Sinusoid = input_array_0[0];
        // moveTo(input_array_0[1], input_array_0[0]);
        // Serial.print("moveTo: ");
        // Serial.print(input_array_0[0]);
        // Serial.print(", ");
        // Serial.println(input_array_0[1]);
      }
      else
      {
        Serial.println("Error: checksum does not match ");
      }
      break;
    case 1: // moveOn
      byte input_array_1[4];
      for (byte i = 0; i < 4; i++)
      {
        input_array_1[i] = Wire.read();
      }

      // Serial.print("moveOn: ");
      // Serial.print(input_array_1[0]);
      // Serial.print(", ");
      // Serial.print(input_array_1[1]);
      // Serial.print(", ");
      // Serial.print(input_array_1[2]);
      // Serial.print(", ");
      // Serial.println(input_array_1[3]);
      if (check_control_summ(sizeof(input_array_1), input_array_1))
      {
        isStartMoveOn = true;
        Direction = input_array_1[1];
        angleOfRotation = input_array_1[2];
        Sinusoid = input_array_1[0];
        // moveOn(input_array_1[0], input_array_1[1], input_array_1[2]);
      }
      else
      {
        // Serial.println("Error: checksum does not match ");
      }
      break;
    case 2: // запись id в eeprom
      byte input_array_2[2];
      for (byte i = 0; i < 2; i++)
      {
        input_array_2[i] = Wire.read();
      }
      if (check_control_summ(sizeof(input_array_2), input_array_2))
      {
        EEPROM.write(0, input_array_2[0]);
        // Serial.print("writeId: ");
        // Serial.println(input_array_2[0]);
      }
      else
      {
        // Serial.println("Error: checksum does not match ");
      }
      break;
    case 3: // запись скорости в eeprom
      byte input_array_3[2];
      for (byte i = 0; i < 2; i++)
      {
        input_array_3[i] = Wire.read();
      }
      if (check_control_summ(sizeof(input_array_3), input_array_3))
      {
        EEPROM.write(1, input_array_3[0]);
        // Serial.print("writeSpeed: ");
        // Serial.println(input_array_2[0]);
      }
      else
      {
        // Serial.println("Error: checksum does not match ");
      }
      break;
    }
  }
}

byte getAngle()
{
  //Получение угла с энкодера
  return byte(expRunningAverage(map(analogRead(A0), 84, 532, 0, 180), 0.1));
}

void requestEvent()
{
  Wire.write(getAngle());
}

void setup()
{
  for (int i = 0; i < 100; ++i)
  {
    getAngle();
  }
  timerfor_moveTo = millis();
  // rotationSpeed = EEPROM.read(1);
  // Wire.begin(EEPROM.read(0));
  // rotationSpeed = 100;
  Wire.begin(8);
  Serial.begin(9600);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  localServo.attach(3);
  Serial.println("0, 0.1, 0.5, 0.9");
}

void loop()
{
  if (isStartMoveTo)
  {
    nowAngle = lastServoAngle;
    isNowMoveTo = true;
    isStartMoveTo = false;
  }
  else if (isStartMoveOn && !isNowMoveTo)
  {

    // Serial.println('a');

    // Serial.println("q[");
    // Serial.print(lastServoAngle);
    // Serial.print(", ");
    // Serial.print(angleOfRotation);
    // Serial.println("]");
    if (((int(lastServoAngle + angleOfRotation) <= 180) && Direction) || ((int(lastServoAngle - angleOfRotation) >= 0) && !Direction))
    {

      // Serial.println('w');
      angleOfRotation = lastServoAngle + (Direction ? (angleOfRotation) : (-angleOfRotation));
      isStartMoveTo = true;
    }
    else
    {
      // Serial.println("Error: uncorrect input data of [moveOn]");
    }

    isStartMoveOn = false;
  }

  if (isNowMoveTo && (millis() - timerfor_moveTo) > 10)
  {

    // Serial.print("---[");
    // Serial.print(1);
    // Serial.println("]---");
    if (nowAngle != angleOfRotation)
    {

      // Serial.print("---[");
      // Serial.print(2);
      // Serial.println("]---");
      if (Sinusoid)
      {
        // Serial.print("---[");
        // Serial.print(3);
        // Serial.println("]---");
        float mini = angleOfRotation > lastServoAngle ? lastServoAngle : angleOfRotation;
        float maxi = angleOfRotation > lastServoAngle ? angleOfRotation : lastServoAngle;
        int a = float((sin(map(float(nowAngle), mini, maxi, float(0), (M_PI * 2)) + (M_PI * 1.5))) + 1) * rotationSpeed * 0.5;
        // Serial.println(a);
        localServo.write(nowAngle, a);
      }
      else
      {
        // Serial.print("---[");
        // Serial.print(4);
        // Serial.println("]---");
        localServo.write(nowAngle, rotationSpeed);
      }
      nowAngle += 1 + (-2 * int(lastServoAngle > angleOfRotation));
    }
    else
    {
      localServo.write(angleOfRotation);
      lastServoAngle = angleOfRotation;
      isNowMoveTo = false;
    }
  }
  /*Serial.print((float)analogRead(A0));
  Serial.print(',');
  Serial.print(expRunningAverage((float)analogRead(A0), 0.1));
  Serial.print(',');
  Serial.print(expRunningAverage((float)analogRead(A0), 0.5));
  Serial.print(',');
  Serial.print(expRunningAverage((float)analogRead(A0), 0.9));
  Serial.println();*/
}

// 180 = 614; 0 = 96
// 180 = 532; 0 = 84;