#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
#include <Wire.h>

#define PCAADDR 0x70

SensirionI2cSf06Lf sensor0;
SensirionI2cSf06Lf sensor1;
SensirionI2cSf06Lf sensor2;

const int P1Pin = 5;
const int P2Pin = 4;
const int PressReg1Pin = 2;
const int PressReg2Pin = 3;
const float gelSeatQm = 0.3;
const float setDp = 2.5;

int currentPWMValue1 = -1;
int currentPWMValue2 = -1;

//Variables below were in setup before
float aFlow = 0.0;
float aTemperature = 0.0;
uint16_t aSignalingFlags = 0u;
//float P1; float P2;
float Q0; float Q1; float Q2;
float Qm = 0.0; float dP = 0.0;
bool dpReached = false; bool gelSeating = true; bool errorFlag = false;
int seatingAttempt = 0; bool evenFlow = false;
int timer1 = 0; int timer2 = 0;

static int16_t error;

void(* resetFunc) (void) = 0;
// call resetFunc(); to reset arduino

void pcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void print_byte_array(uint8_t* array, uint16_t len) {
  uint16_t i = 0;
  Serial.print("0x");
  for (; i < len; i++) {
    Serial.print(array[i], HEX);
  }
}

void setup_flow_sensor(SensirionI2cSf06Lf& sensor, int16_t error, uint8_t t, uint32_t productIdentifier, uint8_t serialNumber[8]) {
  pcaselect(t);
  sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);
  sensor.stopContinuousMeasurement();
  delay(100);
  error = sensor.readProductIdentifier(productIdentifier, serialNumber, 8);
  if (error != NO_ERROR) {
    Serial.print("Q");
    Serial.print(t);
    Serial.println(" sensor failed to start!");
    return;
    //Add code that resets the Arduino, so that it doesnt enter Void loop
  }
  Serial.print("productIdentifier: ");
  Serial.print(productIdentifier);
  Serial.print("\t");
  Serial.print("serialNumber: ");
  print_byte_array(serialNumber, 8);
  Serial.println();
  error = sensor.startH2oContinuousMeasurement();
  if (error != NO_ERROR) {
    Serial.println("Error executing startH2oContinuousMeasurement");
    return;
    //Add code that resets the Arduino, so that it doesnt enter Void loop
  }
  Serial.println("Sensor successfully initialized");
}

float read_p(int pPin) {
  int pValue = analogRead(pPin);
  float pressP = ((pValue * 0.00488758) - 0.5) * 15;
  return pressP;
}

float read_dp() {
  //int P1Value = analogRead(P1Pin);
  //int P2Value = analogRead(P2Pin);
  //float PressP1 = ((P1Value * 0.00488758) - 0.5) * 15;
  //float PressP2 = ((P2Value * 0.00488758) - 0.5) * 15;
  //float dP = PressP1 - PressP2;
  dP = read_p(P1Pin) - read_p(P2Pin);
  return dP;
}

void adjust_dp() {
  dP = read_dp();
  if (dP < (setDp - 0.1) && !errorFlag) {
    if (currentPWMValue1 < 200) {
      currentPWMValue1 = currentPWMValue1 + 1;
      analogWrite(PressReg1Pin, currentPWMValue1);
      currentPWMValue2 = currentPWMValue2 + 1;
      analogWrite(PressReg2Pin, currentPWMValue2);
      delay(200);
      } else {
        errorFlag = true;
        }
  }
  if (dP > (setDp + 0.1) && !errorFlag) {
    if (currentPWMValue1 > 0) {
      currentPWMValue1 = currentPWMValue1 - 1;
      analogWrite(PressReg1Pin, currentPWMValue1);
      currentPWMValue2 = currentPWMValue2 - 1;
      analogWrite(PressReg2Pin, currentPWMValue2);
      delay(200);
      } else {
        errorFlag = true;
        }
  }
}

float read_Q(SensirionI2cSf06Lf& sensor, int16_t error, int16_t t, float Flow) {
  pcaselect(t);
  float Temperature = 0.0;
  uint16_t SignalingFlags = 0u;
  error = sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F,
                                     Flow, Temperature, SignalingFlags);
  if (error != NO_ERROR) {
    Serial.print("Error trying to execute readMeasurementData() on Q");
    Serial.println(t);
    return NAN;
  }
  return Flow;
}

void fast_setup_flow_sensor(SensirionI2cSf06Lf& sensor, uint8_t t) {
  pcaselect(t);
  sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);
  sensor.stopContinuousMeasurement();
  sensor.startH2oContinuousMeasurement();
}

float read_Qm(float Flow) {
  int i = 1;
  while (true) {
    float Mq = (read_Q(sensor0, error, 0, Flow) - read_Q(sensor1, error, 1, Flow)) - read_Q(sensor2, error, 2, Flow);
    if (isnan(Mq) && i < 100) {
    fast_setup_flow_sensor(sensor0, 0);
    fast_setup_flow_sensor(sensor1, 1);
    fast_setup_flow_sensor(sensor2, 2);
    delay(20);
    i++;
    } else {
      return Mq;
      break;
    }
  }
}

void adjust_Q() {
  Q0 = read_Q(sensor0, error, 0, aFlow);
  Q1 = read_Q(sensor1, error, 1, aFlow);
  if (Q1 > (Q0/2)+0.1 && !errorFlag){
    currentPWMValue1 = currentPWMValue1 + 1;
    if (currentPWMValue1 > 200) errorFlag = true;
    analogWrite(PressReg1Pin, currentPWMValue1);
    delay(200);
  }
  if (Q1 < (Q0/2)-0.1 && !errorFlag){
    currentPWMValue1 = currentPWMValue1 - 1;
    if (currentPWMValue1 < 0) errorFlag = true;
    analogWrite(PressReg1Pin, currentPWMValue1);
    delay(200);
  }
}

void print_all_sensor_values() {
  Serial.print("Time: ");
  Serial.print(millis());
  Serial.print("ms, P1: ");
  Serial.print(read_p(P1Pin));
  Serial.print("psi, P2: ");
  Serial.print(read_p(P2Pin));
  Serial.print("psi, dP: ");
  Serial.print(read_dp());
  Serial.print("psi, Q0: ");
  Serial.print(read_Q(sensor0, error, 0, aFlow));
  Serial.print("mL/min, Q1: ");
  Serial.print(read_Q(sensor1, error, 1, aFlow));
  Serial.print("mL/min, Q2: ");
  Serial.print(read_Q(sensor2, error, 2, aFlow));
  Serial.print("mL/min, Qm: ");
  Serial.print(read_Qm(aFlow));
  Serial.println("mL/min");
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
  delay(100);
  }
  Serial.println("Getting Ready...");
  Wire.begin();
  uint32_t productIdentifier = 0;
  uint8_t serialNumber[8] = {0};
  setup_flow_sensor(sensor0, error, 0, productIdentifier, serialNumber);
  setup_flow_sensor(sensor1, error, 1, productIdentifier, serialNumber);
  setup_flow_sensor(sensor2, error, 2, productIdentifier, serialNumber);

  pinMode(P1Pin, INPUT);
  pinMode(P2Pin, INPUT);
  //pinMode(PressReadPin, INPUT);
  pinMode(PressReg1Pin, OUTPUT);
  pinMode(PressReg2Pin, OUTPUT);

  Serial.println("Setup Complete!");
  delay(2000);
}

void loop() {
  while (Serial.available() == 0) {
    //Serial print function for all sensor values
    print_all_sensor_values();
    delay(200);
  }
  String userInput = Serial.readString();
  userInput.trim(); 
  if (userInput == "START" || !gelSeating) {
    //place the code that attains and maintains setDp
    if (seatingAttempt == 3) return;
    do {
      if (!dpReached) adjust_dp();
      if (dP > (setDp - 0.1) && dP < (setDp + 0.1)) dpReached = true;
      if (errorFlag) break;
      if (dpReached && Qm < gelSeatQm) {
        timer2++; timer1++; delay(1000);
        } else {
          timer2 = 0; timer1++; delay(1000);
        }
      if (timer1 > 120) {
        gelSeating = false; Serial.println("Gel did not seat after 3 attempts."); 
        seatingAttempt++; delay(2000); break;
      }
      if (timer2 > 30) {
        gelSeating = true; break;
      }
      //Serial print function for all sensor values
      print_all_sensor_values();
    } while (!dpReached || !gelSeating);
    Serial.println("dP reached and Gel seated! Type READY to go to the next step");
    //code below does the same thing as the code above
    /*
    while (true) {
      adjust_dp();
      if (dp > (setDp - 0.1) && dp < (setDp + 0.1)) {
        //insert some sort of flag that indicates the pressure regulators are adjusted
        break;
      }
    }
    while (true) {
      Qm = read_Qm(aFlow);
      if (Qm < gelSeatQm) {
        timer2++; timer1++; delay(1000);
      } else {
        timer2 = 0; timer1++; delay(1000);
      }
      if (timer1 > 120) {
        gelSeating = false; delay(2000); 
        Serial.println("Gel did not seat."); break;
      }
      if (timer2 > 30) {
        gelSeating = true;
        Serial.println("Gel is seated!");
        break;
      }
    }*/
  }
  if (userInput == "READY") {
    //Place the code that splits the flow at the inlet of Xfer chamber
    do {
      adjust_Q();
      delay(200);
      if (Q1 > (Q0/2)-0.1 && Q1 < (Q0/2)+0.1) evenFlow = true;
      //Serial print function for all sensor values
      print_all_sensor_values();
    } while (!evenFlow);
    Serial.println("Flow is evenly split between gel and non-gel side, Type GO to start transfer");
  }
  if (userInput == "GO") {
    unsigned long timeNow = millis();
    do {
      //keep dp in check
      if (dP < (setDp - 0.1) || dP > (setDp + 0.1)) adjust_dp();
      delay(200);
      if (Q1 < (Q0/2)-0.1 || Q1 > (Q0/2)+0.1) adjust_Q();
      delay(200);
      //Serial print function for all sensor values
      print_all_sensor_values();
    } while (millis() - timeNow < 1200000);
  }
  if (userInput == "RESET") {
    //set pressure regulators to 0 
    resetFunc();
  }
}
