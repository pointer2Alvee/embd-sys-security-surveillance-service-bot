/*3S-BOT
  PREPEARED by team TRI-ANCESSTILLIANTS
  INTELLECTUAL PROPERTY OF TEAM TRI-ANCESTILLIANS
  ALL RIGHTS RESERVED BY TEAM TRI-ANCESTILLIANTS©
  Initial CODE - VERSION-0.1 -11-02-2022-
  Initial CODE - VERSION-0.2 -12-02-2022-
  Initial CODE - VERSION-0.3 -07-03-2022-
  COPY RIGHT ©2022
  180204074-ALVEE (LEAD)
  180204054-ARMAN
  180204073-RAHAT
  170104001-MINHAJUL ISLAM JIM
*/
//*************FEATURES************//
//_____________LIBRARIES___________//
#include<dht.h>
#include <Servo.h>
#include<LiquidCrystal.h>
//_____________VARIABLES___________//

//L298N DC MOTOR VARIABLES
#define enA 2  //Enable1 L298 Pin enA __UNUSED
#define in1 A15 //Motor1  L298 Pin in1 
#define in2 A14 //Motor1  L298 Pin in1 
#define in3 A12 //Motor2  L298 Pin in1 
#define in4 A11 //Motor2  L298 Pin in1 
#define enB 3  //Enable2 L298 Pin enB __UNUSED
int Speed = 75;

//First Time running
int BOT_Initialstart = 1;

//Bluetooth data
int bt_data;

// SERVO PINS
int servoPin = 53; //__UNUSED

//ALARM VARIABLES
//LEDs
byte pin_RedLED = 22;
byte pin_GreenLED = 23;
//Buzzer
byte pin_Buzzer = 24;

//LAMP-LIGHT VARIABLE
byte pin_LAMP = 25;

//FLAME-SENSOR VARIABLES
byte pin_FlameSensor = 15;
byte flameSensorData = 0;

//LDR-SENSOR VARIABLES
byte pin_LDRsensor = 14;
int LDRSensorData;

//DHT11 TEMP-HUMIDITY SENSOR VARIABLES
byte pin_DHT11Sensor = A8;

//MQ-2 GAS/SMOKE SENSOR VARIABLES
byte pin_MQ2Sensor = A7;
int  MQ2SnesorThreshold = 255;
int  MQ2SensorData;

//HCSR04 ULTRASONIC SENSOR VARIABLES
double numberOfMeasurements = 5;
int alertDistance = 19; //cm

//SOUND SENSOR VARIABLES
int pin_soundSensor = 21;

//SENSOR - 1 LEFT __UNUSED
byte pin_TrigUS1 = 26;
byte pin_EchoUS1 = 27;

//SENSOR - 2 MID
byte pin_TrigUS2 = 28;
byte pin_EchoUS2 = 29;

//SENSOR - 3 RIGHT __UNUSED
byte pin_TrigUS3 = 30;
byte pin_EchoUS3 = 31;

// LCD PINS
/* Connect the LCD to pins as shown */
int RS = 7 ;
int EN = 8;
int D4 = 9;
int D5 = 10;
int D6 = 11;
int D7 = 12;

//_____________OBJECTS___________//
dht objectDHT;
Servo thisServo;

//Creating object type - LiquidCrystal
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//_____________FUNCTIONS___________//

//ULTRASONIC'S SERVO __UNUSED
void func_ServoMotor() {
  for (int i = 15; i <= 165; i++) {
    thisServo.write(i);
    delay(10);
  }
  // Repeats the previous lines from 165 to 15 degrees
  for (int i = 165; i >= 15; i--) {
    thisServo.write(i);
    delay(10);
  }
}

//L298N Motor Sheild Function
void func_DCMotorCarSystem(byte state) {

  if (state == 1) {
    //FORWARD
    digitalWrite(in1, HIGH); //Right Motor forword Pin
    digitalWrite(in2, LOW);  //Right Motor backword Pin
    digitalWrite(in3, LOW);  //Left Motor backword Pin
    digitalWrite(in4, HIGH); //Left Motor forword Pin
  } else if (state == 2) {
    //BACKWARD
    digitalWrite(in1, LOW);  //Right Motor forword Pin
    digitalWrite(in2, HIGH); //Right Motor backword Pin
    digitalWrite(in3, HIGH); //Left Motor backword Pin
    digitalWrite(in4, LOW);  //Left Motor forword Pin
  } else if (state == 3) {
    //LEFT
    digitalWrite(in1, HIGH);  //Right Motor forword Pin
    digitalWrite(in2, LOW); //Right Motor backword Pin
    digitalWrite(in3, HIGH);  //Left Motor backword Pin
    digitalWrite(in4, LOW); //Left Motor forword Pin
  } else if (state == 4) {
    //RIGHT
    digitalWrite(in1, LOW); //Right Motor forword Pin
    digitalWrite(in2, HIGH);  //Right Motor backword Pin
    digitalWrite(in3, LOW); //Left Motor backword Pin
    digitalWrite(in4, HIGH);  //Left Motor forword Pin

  } else if (state == 5) {
    //STOP
    digitalWrite(in1, LOW); //Right Motor forword Pin
    digitalWrite(in2, LOW); //Right Motor backword Pin
    digitalWrite(in3, LOW); //Left Motor backword Pin
    digitalWrite(in4, LOW); //Left Motor forword Pin
  }

}

//Bluetooth System Function
void func_BluetoothSystem() {

  if (Serial.available() > 0) {
    bt_data = Serial.read();
    Serial.println(bt_data);
    if (bt_data > 20) {
      Speed = bt_data;
    }
  }
  analogWrite(enA, Speed); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed
  analogWrite(enB, Speed); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed
}

// Car Control System
void func_RCcarControlSystem() {

  func_BluetoothSystem();

  if (bt_data == 1) {
    func_DCMotorCarSystem(1);  // if the bt_data is '1' the DC motor will go forward
  }
  else if (bt_data == 2) {
    func_DCMotorCarSystem(2); // if the bt_data is '2' the motor will Reverse
  }
  else if (bt_data == 3) {
    func_DCMotorCarSystem(3); // if the bt_data is '3' the motor will turn left
  }
  else if (bt_data == 4) {
    func_DCMotorCarSystem(4); // if the bt_data is '4' the motor will turn right
  }
  else if (bt_data == 5) {
    func_DCMotorCarSystem(5);  // if the bt_data '5' the motor will Stop
  }

  else if (bt_data == 6) {
    // func_DCMotorCarSystem(3);
    // delay(400);
    bt_data = 5;
  }
  else if (bt_data == 7) {
    //   func_DCMotorCarSystem(2);
    // delay(400);
    bt_data = 5;
  }
}
//Serial Motion Print For debugging purposes __UNUSED
void func_PrintSerialMonitorSystem() {
  //__UNUSED
}
// Alarm System Function
void func_AlarmSystem(boolean status_ALARM) {
  if (status_ALARM) {
    digitalWrite(pin_RedLED, HIGH);
    digitalWrite(pin_Buzzer, HIGH);
    digitalWrite(pin_GreenLED, LOW);
    delay(205);
    digitalWrite(pin_RedLED, LOW);
    digitalWrite(pin_Buzzer, LOW);
    digitalWrite(pin_GreenLED, LOW);
    delay(205);
  } else if (!status_ALARM) {
    digitalWrite(pin_GreenLED, HIGH);
    digitalWrite(pin_RedLED, LOW);
    digitalWrite(pin_Buzzer, LOW);
    delay(250);
  }

}

//LAMP System Function
void func_LAMPsystem(boolean status_LAMP) {
  if (status_LAMP) {
    digitalWrite(pin_LAMP, HIGH);

  } else if (!status_LAMP) {
    digitalWrite(pin_LAMP, LOW);

  }
}

//Flame Detection Function
void func_FlameDetection() {
  func_RCcarControlSystem() ;

  flameSensorData = digitalRead(pin_FlameSensor);
  if (flameSensorData == 0) {
    func_AlarmSystem(true);

    //Display at LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FIRE DETECTED !");
    Serial.println("FIRE");
    delay(500);
    lcd.clear();

  } else if (flameSensorData == 1) {
    func_AlarmSystem(false);
  }
}

//LDR Detection Function
void func_LightDetection() {
  func_RCcarControlSystem() ;
  LDRSensorData = digitalRead(pin_LDRsensor);
  if (LDRSensorData == 1) {
    Serial.println("lampOn");
    func_LAMPsystem(true);

    //Display at LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LAMP ON !");
    delay(500);
    lcd.clear();

  } else if (LDRSensorData == 0) {
    Serial.println("lampOff");
    func_LAMPsystem(false);
  }
}

//DHT11 Temp-Humidity Function
void func_TempHumidDetection() {
  func_RCcarControlSystem() ;

  objectDHT.read11(pin_DHT11Sensor);
  //prints results
  Serial.print("TEMPERATURE: ");
  Serial.print(objectDHT.temperature);
  Serial.print("C: ");
  Serial.print("     ");
  Serial.print("HUMIDITY: ");
  Serial.print(objectDHT.humidity);
  Serial.print("%");

  //Display at LCD
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("TEMP: ");
  lcd.print(objectDHT.temperature);
  lcd.print(" C");

  lcd.setCursor(0, 1);
  lcd.print("HUMID: ");
  lcd.print(objectDHT.humidity);
  lcd.print(" %");
  delay(500);
  lcd.clear();

}

//MQ2 Gas-Smoke  Function
void func_GasSmokeDetection() {
  func_RCcarControlSystem() ;

  MQ2SensorData = analogRead(pin_MQ2Sensor);
  Serial.print("     ");
  Serial.print("mq2 Sensor Data: ");
  Serial.println(MQ2SensorData);
  if (MQ2SensorData > MQ2SnesorThreshold) {
    func_AlarmSystem(true);

    //Display at LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Gas Data:");
    lcd.print(MQ2SensorData);
    delay(500);

  } else if (MQ2SensorData < MQ2SnesorThreshold) {
    func_AlarmSystem(false);
  }
}

//Sound Sensor Function
void func_SoundDetection() {
  func_RCcarControlSystem() ;

  int statusSensor = digitalRead (pin_soundSensor);
  if (statusSensor == 1) {
    func_AlarmSystem(true);
    Serial.print("SOUND");

    //Display at LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("UNWANTED");
    lcd.setCursor(0, 1);
    lcd.print("SOUND DETECTED!");
    delay(700);

  } else {
    func_AlarmSystem(false);
  }
}

//ULTRASONIC Object detection Function
void func_ObjectDetection() {
  func_RCcarControlSystem() ;
  //US-1 *****_UNUSED*****
  double US1_signalTravelTime;
  double US1_distance; // core mathematical calculations
  double US1_sumDistance = 0; // summation for all numberOfMeasurements
  double US1_averageDistance;

  //US-2
  double US2_signalTravelTime;
  double US2_distance; // core mathematical calculations
  double US2_sumDistance = 0; // summation for all numberOfMeasurements
  double US2_averageDistance;

  //US-3 *****_UNUSED*****
  double US3_signalTravelTime;
  double US3_distance; // core mathematical calculations
  double US3_sumDistance = 0; // summation for all numberOfMeasurements
  double US3_averageDistance;


  ///for accurate data and eliminating noise e use for loop upto numberOfMeasurements
  for ( int i = 0; i < numberOfMeasurements; i++) {
    //digitalWrite(pin_TrigUS1, LOW);
    digitalWrite(pin_TrigUS2, LOW);
    //digitalWrite(pin_TrigUS3, LOW);

    delayMicroseconds(2);
    //digitalWrite(pin_TrigUS1, HIGH);
    digitalWrite(pin_TrigUS2, HIGH);
    //digitalWrite(pin_TrigUS3, HIGH);

    delayMicroseconds(10);
    //digitalWrite(pin_TrigUS1, LOW);
    digitalWrite(pin_TrigUS2, LOW);
    // digitalWrite(pin_TrigUS3, LOW);


    //US1_signalTravelTime = pulseIn(pin_EchoUS1, HIGH); // returns in microseconds
    US2_signalTravelTime = pulseIn(pin_EchoUS2, HIGH); // returns in microseconds
    //  US3_signalTravelTime = pulseIn(pin_EchoUS3, HIGH); // returns in microseconds
    delay(10);

    // 0.40 is the faulty value of the US sensor i've used
    // US1_distance =  (((0.01350391 * US1_signalTravelTime) / 2) * 2.54) + 0.4; // Outputs in CENTIMETERS
    US2_distance =  (((0.01350391 * US2_signalTravelTime) / 2) * 2.54) + 0.4; // Outputs in CENTIMETERS
    //US3_distance =  (((0.01350391 * US3_signalTravelTime) / 2) * 2.54) + 0.4; // Outputs in CENTIMETERS


    US1_sumDistance = US1_sumDistance + US1_distance;
    US2_sumDistance = US2_sumDistance + US2_distance;
    US3_sumDistance = US3_sumDistance + US3_distance;

  }

  // US1_averageDistance = (US1_sumDistance / numberOfMeasurements);
  US2_averageDistance = (US2_sumDistance / numberOfMeasurements);
  // US3_averageDistance = (US3_sumDistance / numberOfMeasurements);

  if ( US2_distance < alertDistance ) {

    Serial.print("Object at FRONT less than : ");
    Serial.print(US2_distance);
    Serial.print(" cm");
    Serial.println("");

    //Display at LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OBJECT AT FRONT: ");
    lcd.setCursor(0, 1);
    lcd.print(US2_distance);
    lcd.print(" cm");
    delay(1500);
    lcd.clear();

    func_DCMotorCarSystem(5);
    delay(1000);
    func_DCMotorCarSystem(2);
    delay(1000);
    func_DCMotorCarSystem(5);

    //    if ((US3_distance > alertDistance)) {
    //      Serial.print("RIGHT CLEAR ");
    //      Serial.print(US3_distance);
    //      Serial.print(" cm");
    //      Serial.println("");
    //      func_DCMotorCarSystem(4);
    //      delay(500);
    //      func_DCMotorCarSystem(5);
    //    } else if ((US1_sumDistance > alertDistance)) {
    //      Serial.print("LEFT CLEAR: ");
    //      Serial.print(US1_sumDistance);
    //      Serial.print(" cm");
    //      Serial.println("");
    //      func_DCMotorCarSystem(3);
    //      delay(500);
    //      func_DCMotorCarSystem(5);
    //    } else {
    //      Serial.print("BACK CLEAR: ");
    //      Serial.print(US1_sumDistance);
    //      Serial.print(" cm");
    //      Serial.println("");
    //      func_DCMotorCarSystem(2);
    //      delay(500);
    //      func_DCMotorCarSystem(5);
    //    }
    func_AlarmSystem(true);
  } else {
    func_AlarmSystem(false);
  }

  // 0.40 is the faulty value of the US sensor i've used
  US1_sumDistance = 0; // clears the previous data . orelse it will keep adding
  US2_sumDistance = 0; // clears the previous data . orelse it will keep adding
  US3_sumDistance = 0; // clears the previous data . orelse it will keep adding

}

void welcomeScreen() {
  //FIRST TIME LCD WELCOME SCREEN
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WELCOME! ALVEE");
  lcd.setCursor(0, 1);
  lcd.print("3S-BOT ACTIVATED");
  delay(5000);
  lcd.clear();
}

//SETUP FUNCTION
void setup() {
  // put your setup code here, to run once:
  //Serial Monitor Initi
  Serial.begin(9600);

  //L298N DC Motor Sheild Pins
  pinMode(enA, OUTPUT); // declare as output for L298 Pin enA
  pinMode(in1, OUTPUT); // declare as output for L298 Pin in1
  pinMode(in2, OUTPUT); // declare as output for L298 Pin in2
  pinMode(in3, OUTPUT); // declare as output for L298 Pin in3
  pinMode(in4, OUTPUT); // declare as output for L298 Pin in4
  pinMode(enB, OUTPUT); // declare as output for L298 Pin enB

  //Leds Pin
  pinMode(pin_RedLED, OUTPUT);
  pinMode(pin_GreenLED, OUTPUT);
  pinMode(pin_Buzzer, OUTPUT);
  //LAMP Leds pin
  pinMode(pin_LAMP, OUTPUT);
  //Flame Sensor Pin
  pinMode(pin_FlameSensor, INPUT);
  //LDR Sensor Pin
  pinMode(pin_LDRsensor, INPUT);
  //MQ-2 Sensor Pin
  pinMode(pin_MQ2Sensor, INPUT);
  //Sound Sensor pin
  pinMode(pin_soundSensor, INPUT);
  //Ultrasonic Sensor pins
  //us-1
  pinMode(pin_TrigUS1, OUTPUT);
  pinMode(pin_EchoUS1, INPUT);
  //us-2
  pinMode(pin_TrigUS2, OUTPUT);
  pinMode(pin_EchoUS2, INPUT);
  //us-3
  pinMode(pin_TrigUS3, OUTPUT);
  pinMode(pin_EchoUS3, INPUT);

  //Ultrasonic-servo
  thisServo.attach(servoPin); // Defines on which pin is the servo motor attached

  //LCD Display
  lcd.begin(16, 2);   // set-up LCD display
}


//LOOP FUNCTION
void loop() {
  // put your main code here, to run repeatedly:
  if (BOT_Initialstart == 1) {
    welcomeScreen();
    BOT_Initialstart = 0;
  };
  func_RCcarControlSystem() ;
  func_FlameDetection();
  func_TempHumidDetection();
  func_GasSmokeDetection();
  func_LightDetection();
  func_ObjectDetection();
  func_SoundDetection();
  //func_ServoMotor();


}
