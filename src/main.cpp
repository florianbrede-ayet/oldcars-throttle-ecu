#include <Arduino.h>
#include <CAN.h>

/* V1.0 BMW Actuator ECU throttle
This sketch can be used to control a BMW throttle valve actuator over CAN. 

original source: https://github.com/Lukilink/actuator_ECU
*/


// ACTUATOR SETUP:
//________________THIS IS THE CONFIGURATION FOR A BMW/VDO "8 369 027" / "408.201/013/001" ACTUATOR with 30mm actuation distance
int PERM_ERROR = 25; //will allow a diffrence between targetPressure and currentPressure
int minPot = 20; //measured at actuators lowest position
int maxPot = 2300; //measured at actuators highest position
const int SLOW_MOVE_PWM=200;

// ACTUATOR POTENTIOMETER:
int potPin = A3; // connect the potentiometer of your cars throttle
int potReferenceResistor = 1000; // we measure the resistance of the potentiometer - this is the reference resistor used to cover the ~2.4k ohms potentiometer range

unsigned long lastCanReceive=0L;


// SAFETY PINS
int cancel_pin_clutch = 3; //pulled to GND when CLUTCH pedal pressed
int cancel_pin_brake = 4; //pulled to GND when BRAKE pedal pressed
unsigned long lastClutchPressedTime = 0L;

const int CLUTCH_RELEASE_GRACE_TIME_MS = 500; // amount of ms to delay giving throttle after shifting for example

// H BRIDGE PINS
int M_IN1 = 7; // motor direction (7H, 8L = left, 7L, 8H = right)
int M_IN2 = 8; // motor direction (7H, 8L = left, 7L, 8H = right)
int M_ENA = 9; // 255 is run / LOW is stopp   // motor speed
// we bridge ENB and just control IN3 to set the clutch closed or to 0 again to open
int S_IN3 = 6; // 255 is run / LOW is stopp   // SOLENOID / actuator clutch!

// OPENPILOT CONFIG 
float maxACC_CMD = 1430; //the max Value which comes from OP on CAN ID 0x200
float minACC_CMD = 477; //the min Value which comes from OP on CAN ID 0x200


//________________values
int targetPosition = 0;
int potiPosition = 0;
float ACC_CMD_PERCENT = 0;
float ACC_CMD = 0;
float ACC_CMD1 = 0;
boolean cancel = false;



int getResistance() {
  int potRaw = analogRead(potPin);
  if(potRaw) 
  {
    float buffer=potRaw * 5.0f;
    float Vout = (buffer)/1024.0;
    buffer = (5.0f/Vout) - 1;
    return (int)(potReferenceResistor * buffer);
  }
  return 0;
}

void setup() {
  //________________begin Monitor - only use it for debugging
  Serial.begin(115200);
  CAN.setClockFrequency(8E6);

  //________________begin CAN
  CAN.begin(500E3);

  CAN.filter(0x200);
  
  //________________set up pin modes
  pinMode(cancel_pin_clutch, INPUT_PULLUP);
  pinMode(cancel_pin_brake, INPUT);
  pinMode(potPin, INPUT);    
  
  pinMode(M_IN1, OUTPUT);
  pinMode(M_IN2, OUTPUT);
  pinMode(M_ENA, OUTPUT);
  digitalWrite(M_IN1, LOW);
  digitalWrite(M_IN2, LOW);
  digitalWrite(M_ENA, LOW);

  pinMode(S_IN3, OUTPUT);
  digitalWrite(S_IN3, LOW);


  

  // UNCOMMENT FOR DEBUGGING
  /* Serial.println("OPEN / STOP");
  getResistance();
  while(1) {
    digitalWrite(S_IN3, HIGH);
    Serial.println("CLOSE SOLENOID");
    delay(5000);

    digitalWrite(M_IN1, HIGH); //motor driection left
    digitalWrite(M_IN2, LOW); //motor driection left
    
    analogWrite(M_ENA, 255);  //run Motor
    Serial.println("RUN MOTOR LEFT until end");
    while(1) {
      int res = getResistance();
      if (res>=maxPot) break;
      delay(1);
    }
    analogWrite(M_ENA, 0);  //stop Motor
    Serial.print("FINAL RESISTANCE  ");
    Serial.println(getResistance());
    delay(1000);
    Serial.print("FINAL RESISTANCE  AFTER 1s sleep   ");
    Serial.println(getResistance());
    delay(1000);


    digitalWrite(M_IN1, LOW); //motor driection right
    digitalWrite(M_IN2, HIGH); //motor driection right
    analogWrite(M_ENA, 255);  //run Motor
    Serial.println("RUN MOTOR RIGHT until end");
    while(1) {
      int res = getResistance();
      if (res<=minPot) break;
      delay(1);
    }
    delay(5000);
  
    Serial.print("END CYCLE RESISTANCE  ");
    Serial.println(getResistance());
  } */

}

void loop() {
  //________________check cancel conditions

  if (!digitalRead(cancel_pin_clutch))  // we implement a clutch-grace time to make sure we don't overrev the engine while releasing the clutch slowly
    lastClutchPressedTime=millis();

  cancel = (!digitalRead(cancel_pin_clutch) || millis()-lastClutchPressedTime<CLUTCH_RELEASE_GRACE_TIME_MS || !digitalRead(cancel_pin_brake) || millis()-lastCanReceive>500 || millis()<lastCanReceive);

  //________________read poti Position
  int potiPosition = (int)(((float)(getResistance()+getResistance()+getResistance()+getResistance()+getResistance()))/5.0);


  //________________read ACC_CMD from CANbus
  CAN.parsePacket();
  Serial.println(CAN.packetId());
  
  if (CAN.packetId() == 0x200) {
    lastCanReceive=millis();
    uint8_t dat[8];
    for (int ii = 0; ii <= 7; ii++) {
      dat[ii]  = (char) CAN.read();
    }
    ACC_CMD = (dat[0] << 8 | dat[1] << 0); 
  } 
 
  //________________calculating ACC_CMD into ACC_CMD_PERCENT
  if (ACC_CMD >= minACC_CMD) {
    ACC_CMD1 = ACC_CMD;
  }
  else {
    ACC_CMD1 = minACC_CMD;
  }


  ACC_CMD_PERCENT = ((100/(maxACC_CMD - minACC_CMD)) * (ACC_CMD1 - minACC_CMD));

  //________________calculating ACC_CMD_PERCENT into targetPosition 
  float targetPosition = (((ACC_CMD_PERCENT / 100) * (maxPot - minPot)) + minPot);

  //________________do nothing if ACC_CMD is 0
  if (ACC_CMD_PERCENT == 0){
    analogWrite(S_IN3, 0);  //open solenoid
    analogWrite(M_ENA, 0);  //stop Motor
  }

  //________________do nothing while cancel, but read if it's still cancel
  if (cancel) {
    analogWrite(S_IN3, 0);  //open solenoid
    analogWrite(M_ENA, 0);  //stop Motor
  }
  else { // we're not braking or changing gears / we can actuate
    //________________close solenoid
    digitalWrite(S_IN3, HIGH);

    //________________press or release the pedal to match targetPosition & respect endpoints
    if (abs(potiPosition - targetPosition) >= PERM_ERROR)
    {
      if ((potiPosition < targetPosition) && (potiPosition < maxPot)) //if we are lower than target and not at endpoint
      { 
        digitalWrite(M_IN1, HIGH); digitalWrite(M_IN2, LOW); //motor left (== pull wire)
        if (abs(potiPosition - targetPosition)>100)
          analogWrite(M_ENA, 255);  //run Motor
        else
          analogWrite(M_ENA, SLOW_MOVE_PWM);  //run Motor
      }    
      else if ((potiPosition > targetPosition) && (potiPosition > minPot)) //if we are higher than target and not at endpoint
      {       
        digitalWrite(M_IN1, LOW); digitalWrite(M_IN2, HIGH); //motor right (== loosen wire)
        if (abs(potiPosition - targetPosition)>100)
          analogWrite(M_ENA, 255);  //run Motor
        else
          analogWrite(M_ENA, SLOW_MOVE_PWM);  //run Motor
      }
      else {
        analogWrite(M_ENA, 0);   //stop Motor (out of bounds stop)
      }
    }  
    //________________if we match target position, just stay here
    else {
      analogWrite(M_ENA, 0);   //stop Motor
    }  
  }
  //________________print stuff if you want to DEBUG
  
  Serial.print("ACC_CMD_");
  Serial.print(ACC_CMD);
  Serial.print("_____");
  Serial.print(ACC_CMD_PERCENT);
  Serial.print("_%");
  Serial.print("_____");
  Serial.print("target_");
  Serial.print(targetPosition);
  Serial.print("_____");
  Serial.print("Position_");
  Serial.print(potiPosition);
  Serial.print("_____");
  Serial.print("offset_");
  Serial.print(abs(targetPosition-potiPosition));
  Serial.print("_____");
  Serial.print("clutch_");
  Serial.print(digitalRead(cancel_pin_clutch));
  Serial.print("_____");
  Serial.print("brake_");
  Serial.print(digitalRead(cancel_pin_brake));
  Serial.print("_____");
  Serial.print("cants_");
  Serial.print(millis()-lastCanReceive);
  Serial.print("_____");
  Serial.print("clutchts_");
  Serial.print(millis()-lastClutchPressedTime);
  Serial.println("");
 

}