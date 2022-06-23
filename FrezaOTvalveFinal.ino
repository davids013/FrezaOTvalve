#include <PIDController.h>
#include <LiquidCrystal.h>
#include <PWM.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

PIDController pid;
#define __kP 24680
#define __kI 50
#define __kD 0
#define __fastSensorPinRead bitRead(PINC, 7)

struct Keypad {
  const byte KEY_NONE = 0;
  const byte KEY_SELECT = 1;
  const byte KEY_LEFT = 2;
  const byte KEY_UP = 3;
  const byte KEY_DOWN = 4;
  const byte KEY_RIGHT = 5;
};

const bool isPropValveUsing = true;
const unsigned int maxADCcount = 1023;
const unsigned int pwmFrequency = 20000;
const float powerVoltage = 5.0;
const float printRate = 5;
const byte maxFrequency = 180;
const byte buttonAnalogPin = 0;
const byte relayPin = 40;   //PG1
const byte neighRelayPin = relayPin + 1;
const byte pwmPin = 12;
const byte sensorPin = 30;  //PC7
const byte potPin = 15;  // analog pin used to connect the potentiometer
//Параметры ниже актуальны для пропорционального клапана SMC PVQ33-6G23-01
const byte nomValveFlow = 110; 
const float nomValvePressure = 0.35;
const float valvePressure = 0.35;
const byte maxValveFlow = 1.0 * nomValveFlow / nomValvePressure * valvePressure; 
const unsigned int minValveCurrent = 160;
const unsigned int maxValveCurrent = 320;
const byte minValvePwmDuty = 126;
const byte maxValvePwmDuty = 245;
const byte stepSize = 1;
const int movDelayUS = 0;
const Keypad keypad;

int val;                // variable to read the value from the analog pin
unsigned long lastTime = 0;
byte curPosition = minValvePwmDuty;
float targetFreq = 1;
float frequency = 0.0;
double lastFreq = 0.0;
bool isRelayOpen = true;

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT_PULLUP);
//  if (isPropValveUsing) {
    pinMode(pwmPin, OUTPUT);
    InitTimersSafe();
    SetPinFrequencySafe(pwmPin, pwmFrequency);
    digitalWrite(pwmPin, HIGH);
//  } else {
    pinMode(relayPin, OUTPUT);
    pinMode(neighRelayPin, INPUT);
    digitalWrite(relayPin, isRelayOpen);
//  }
  int temp = analogRead(potPin);
  targetFreq = map(temp, 0, maxADCcount, 0, maxFrequency * 10) / 10.0;
  targetFreq = 0;
  pid.begin();
  pid.limit(minValvePwmDuty, maxValvePwmDuty);
  pid.tune(__kP, __kI, __kD);
  pid.setpoint(minValvePwmDuty);
  lcd.begin(16, 2);
  lcd.print("Waiting");
  lcd.setCursor(0, 0);
}

byte getKey() {
  int input = analogRead(buttonAnalogPin);
  if (input < 100) {
    return keypad.KEY_RIGHT;
  } else if (input < 300) {
    return keypad.KEY_UP;
  } else if (input < 500) {
    return keypad.KEY_DOWN;
  } else if (input < 700) {
    return keypad.KEY_LEFT;
  } else if (input < 900) {
    return keypad.KEY_LEFT;
  } else return keypad.KEY_NONE;
}

unsigned int currentFromPwmDuty(byte pwmDuty) {
  const float maxDuty = 255.0;
  return pwmDuty / maxDuty * maxValveCurrent;
}

byte flowFromCurrent(int current) {
  if (current > minValveCurrent) {
    return (float) maxValveFlow / (maxValveCurrent - minValveCurrent) * (current - minValveCurrent);
  } else return 0;
}

float getFrequency(int pin, unsigned long timeout) {
  //lastFreq = frequency;
  //bool isStuck = false;
  float f = 0;
  unsigned long tempTime = 0;
  unsigned long timer = micros();
  while (!__fastSensorPinRead) { 
    tempTime = micros();
    if (tempTime > (timer + timeout)) return f; 
  }
  while (__fastSensorPinRead) {
    tempTime = micros();
    if (tempTime > (timer + timeout)) return f; 
  }
  timer = micros();
  while (!__fastSensorPinRead) { }
  while ( __fastSensorPinRead) { }
  timer = micros() - timer;
  const int measureDelay = 0; //1345   //140
  f = 1000000.0 / (timer + measureDelay);
//  if (highTime > 0 && lowTime > 0) frequency = 1000000.0 / (highTime + lowTime);
  if ((frequency > 0) && (f > maxFrequency * 1.5) && (f > frequency * 1.5)) {
    return frequency;
  } else return f;
}

byte propValveControl(float frequency, float targetFreq) {
//    if (frequency > targetFreq && curPosition > minValvePwmDuty) {
//      if (curPosition >= minValvePwmDuty + stepSize) {
//        curPosition -= stepSize;
//      } else curPosition -= 1;
//    } else if (frequency < targetFreq && curPosition < maxValvePwmDuty) {
//        if (curPosition <= maxValvePwmDuty - stepSize) {
//          curPosition += stepSize;
//        } else curPosition += 1;
//    }
    //byte key = getKey();
    //if (key == keypad.KEY_UP && curPosition < maxValvePwmDuty) {
    //  curPosition += 2;
    //  Serial.println("UP");
    //} else if (key == keypad.KEY_DOWN && curPosition > minValvePwmDuty) {
    //  curPosition -= 2; 
    //  Serial.println("DOWN");
    //}
    curPosition = pid.compute(frequency);
    if (curPosition > minValvePwmDuty) {
      pwmWrite(pwmPin, curPosition);
      //analogWrite(pwmPin, curPosition);
    } else pwmWrite(pwmPin, 0); 
           //digitalWrite(pwmPin, LOW);
    delayMicroseconds(movDelayUS); 
    return curPosition;
}

//void propValveControlRes_1(float frequency, float lastFrequency, byte curPosition) {
//  
//}

void loop() {
  int temp = analogRead(potPin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(temp, 0, maxADCcount, minValvePwmDuty, maxValvePwmDuty);     // scale it to use it with the servo (value between 0 and 180)
  float f = getFrequency(sensorPin, 900000);
  //if ((frequency < 10 && f < 10) || (abs(f - frequency) < frequency * 0.3)) {
  frequency = f;
  //}
  //frequency = 80.0;
  f = map(temp, 0, maxADCcount, 0, maxFrequency * 10) / 10.0;
  if (f != targetFreq) {
    targetFreq = f;
    pid.setpoint(targetFreq);
  }
//  if (!isPropValveUsing) {
    bool b = (frequency >= targetFreq) ? false : true;
    if (b != isRelayOpen) {
      isRelayOpen = b;
      digitalWrite(relayPin, isRelayOpen);
    }
//  } else {
    curPosition = propValveControl(frequency, targetFreq);  
//  }
  if (millis() - lastTime > (1000 / printRate)) {
    lastTime = millis();
    Serial.print(frequency);
    Serial.print(" Hz (tar. ");
    Serial.print(targetFreq);
    Serial.print(" Hz)");
    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("                ");
    lcd.setCursor(0, 0);
    lcd.print("f =_");
    lcd.print(frequency, 1);
    lcd.print("Hz_");
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    lcd.print("g =(");
    lcd.print(targetFreq, 1);
    lcd.print("Hz)");
//    if (isPropValveUsing) {
//      int current = currentFromPwmDuty(curPosition);
//      byte flow = flowFromCurrent(current);
//      Serial.print(" -> CurPos = ");
//      Serial.print(curPosition);
//      Serial.print(" -> current = ");
//      Serial.print(current);
//      Serial.print(", flow = ");
//      Serial.print(flow);
//      lcd.setCursor(12, 1);
//      lcd.print("F");
//      lcd.print(flow);
//      lcd.print(current);
//      lcd.print(curPosition);
//    } else {
      Serial.print(" -> Relay = ");
      Serial.print(isRelayOpen);
      lcd.setCursor(14, 0);
//      lcd.print('R'); 
      lcd.print(isRelayOpen);
//    }
//  Serial.println();
  }
}
