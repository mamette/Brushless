//membaca Accelerometer BMA180 dan Gyroscope ITG3205
#include <Wire.h> //The I2C library


// constants won't change. They're used here to 
// set pin numbers:
const int motorPin1 =3;
const int motorPin2 =6;
const int motorPin3 =5;

// Variables will change:
boolean direct = true; // direction true=forward, false=backward

int pwmSin[] = {127,110,94,78,64,50,37,26,17,10,4,1,0,1,4,10,17,26,37,50,64,78,94,110,127,144,160,176,191,204,217,228,237,244,250,253,254,253,250,244,237,228,217,204,191,176,160,144,127
}; // array of PWM duty values for 8-bit timer - sine function

 
//int pwmSin[]={511,444,379,315,256,200,150,106,68,39,17,4,0,4,17,39,68,106,150,200,256,315,379,444,511,578,643,707,767,822,872,916,954,983,1005,1018,1022,1018,1005,983,954,916,872,822,767,707,643,578,511
//}; // array of PWM duty values for 10-bit timer - sine function
 
int increment;
int currentStepA=0;
int currentStepB=16;
int currentStepC=32;

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
//long motorDelayActual = 0;  // the actual delay, based on pot value and motor delay set above
long lastMotorDelayTime = 0;


#define BMA180              0x40
#define ee_w_MASK           0x10
#define mode_config_MASK    0x03
#define bw_MASK             0xF0
#define range_MASK          0x0E
#define lat_int_MASK        0x01
#define lat_int             0x01

int gyroResult[3], AccelX, AccelY, AccelZ, temp;
float timeStep = 0.02;          //20ms. Need a time step value for integration of gyro angle from angle/sec
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro = 0;
float pitchAccel = 0;
float pitchPrediction = 0; //Output of Kalman filter
float rollGyro = 0;
float rollAccel = 0;
float rollPrediction = 0;  //Output of Kalman filter
float yawGyro = 0;
float giroVar = 0.1; //0.1
float deltaGiroVar = 0.1; //0.1
float accelVar = 10; //5
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; //0.1 angle and angle change rate covariance
float kx, kv;

float pitchServo;
float rollServo;

unsigned long timer;

  
//Penjabaran fungsi writeTo sebagai Fungsi untuk writing byte ke alamat device pada I2C
void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

//Penjabaran fungsi readFrom sebagai Fungsi untuk membaca num bytes dari alamat pada device I2C
void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

//Fungsi untuk mmebaca nilai Gyro
void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1]; //Combine two bytes into one int
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

//Accelerometer Init
byte initializeBMA180()
{
  /*Set EEPROM image to write mode so we can change configuration*/
  delay(20);
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte ee_w = Wire.read();
  ee_w |= ee_w_MASK;
  Wire.beginTransmission(BMA180);
  Wire.write(0x0D);
  Wire.write(ee_w);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set mode configuration register to Mode 00*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte mode_config = Wire.read();
  mode_config &= ~(mode_config_MASK);
  Wire.beginTransmission(BMA180);
  Wire.write(0x30);
  Wire.write(mode_config);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set bandwidth to 10Hz*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte bw = Wire.read();
  bw &= ~(bw_MASK);
  bw |= 0x00 << 4;
  Wire.beginTransmission(BMA180);
  Wire.write(0x20);
  Wire.write(bw);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set acceleration range to 2g*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte range = Wire.read();
  range &= ~(range_MASK);
  range |= 0x00 << 1 ;
  /*    case B000: // 1g
        case B001: // 1.5g
        case B010/0x02: // 2g
        case B011: // 3g
        case B100: // 4g
        case B101: // 8g
        case B110: // 16g
        */
  
  Wire.beginTransmission(BMA180);
  Wire.write(0x35);
  Wire.write(range);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  /*Set interrupt latch state to non latching*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte latch_int = Wire.read();
  latch_int &= ~(0x01);
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(latch_int);
  if(Wire.endTransmission()){return(1);}
  delay(20); 
  /*Set interrupt type to new data*/
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,1) != 1){return(2);}
  byte int_type = Wire.read();
  int_type |= 0x02;
  Wire.beginTransmission(BMA180);
  Wire.write(0x21);
  Wire.write(int_type);
  if(Wire.endTransmission()){return(1);}
  delay(20);
  return(0);
}


void gyroInit() {
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
}

void accelerometerGyroBias() {
  
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  int i;
  
  // Determine zero bias for all axes of both sensors by averaging 50 measurements
  for (i = 0; i < 50; i += 1) {
    getGyroscopeReadings(gyroResult);
    readAccel();
    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    totalAccelXValues += AccelX;
    totalAccelYValues += AccelY;
    totalAccelZValues += AccelZ;
    delay(50);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  biasAccelZ = (totalAccelZValues / 50) - 256; //Don't compensate gravity away! We would all (float)!
}

void initBLDC() {

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  
  //TCCR1B = TCCR1B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 9 and 10
  TCCR2B = TCCR2B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 11 and 3 (3 not used)
  //TCCR0B = TCCR0B & 0b11111000 | 0x01; // set PWM frequency @ 31250 Hz for Pins 5 and 6
  
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  //TCCR2B = _BV(CS20);

  ICR1 = 255 ; // 8 bit resolution
  //ICR1 = 1023 ; // 10 bit resolution
  //TIMSK1 |= _BV(TOIE1);
  //sei();

  // Enable Timer1 Interrupt for Motor Control
  //OCR2A = 0;  //11  APIN
  //OCR2B = 0;  //D3
  
}


void setup() {
  
  Wire.begin();            //Open I2C communications as master
  Serial.begin(115200);    //Open serial communications to the PC to see what's happening
  
  gyroInit();
  initializeBMA180();

  delay(100); //wait for gyro to "spin" up
  
  accelerometerGyroBias();
  initBLDC();
}

void loop() {
   
  timer = millis(); //get a start value to determine the time the loop takes
  getGyroscopeReadings(gyroResult);
  readAccel();

  calculation();
  kalmanFilter();
  
  derajatServo();
  printData();
  
  BLDCmove();
 
   //delay(1);

  timer = millis() - timer;          //how long did the loop take?
  timer = (timeStep * 1000) - timer; //how much time to add to the loop to make it last time step msec
  delay(timer);                                    //make one loop last time step msec
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
}


void BLDCmove() {
if((millis() - lastMotorDelayTime) > 0)
{ // delay time passed, move one step 
 
  if (direct==true)
  {
    increment = 1;
    
  currentStepA = currentStepA + increment;
  if(currentStepA > 47) currentStepA = 0;
  if(currentStepA<0) currentStepA =47;
   
  currentStepB = currentStepB + increment;
  if(currentStepB > 47) currentStepB = 0;
  if(currentStepB<0) currentStepB =47;
   
    currentStepC = currentStepC + increment;
  if(currentStepC > 47) currentStepC = 0;
  if(currentStepC<0) currentStepC =47;
  
  } 
   
   
  if (direct==false)
  {
    increment = -1;
    
  currentStepA = currentStepA + increment;
  if(currentStepA > 47) currentStepA = 0;
  if(currentStepA<0) currentStepA =47;
   
  currentStepB = currentStepB + increment;
  if(currentStepB > 47) currentStepB = 0;
  if(currentStepB<0) currentStepB =47;
   
    currentStepC = currentStepC + increment;
  if(currentStepC > 47) currentStepC = 0;
  if(currentStepC<0) currentStepC =47;
  } 
  
lastMotorDelayTime = millis();

}
   
analogWrite(motorPin1, pwmSin[currentStepA]);
analogWrite(motorPin2, pwmSin[currentStepB]);
analogWrite(motorPin3, pwmSin[currentStepC]);
   
}

//*********************************************Function on the Loop Section*********************************************************************************//
byte readAccel()
{
  Wire.beginTransmission(BMA180);
  Wire.write(0x02);
  if(Wire.endTransmission()){return(1);}
  if(Wire.requestFrom(BMA180,7) != 7){return(2);}
  AccelX = Wire.read();
  AccelX |= Wire.read() << 8;
  AccelX >>= 2;

  
  AccelY = Wire.read();
  AccelY |= Wire.read() << 8;
  AccelY >>= 2;

  
  AccelZ = Wire.read();
  AccelZ |= Wire.read() << 8;
  AccelZ >>= 2;
  temp = Wire.read();
  
        if (AccelZ < 1500){
    AccelZ = 1500;}
}


void calculation() {
  pitchAccel = atan2((AccelY - biasAccelY) / 8192, (AccelZ - biasAccelZ) / 8192) * 360.0 / (2*PI);
  pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  pitchPrediction = pitchPrediction + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  
  rollAccel = atan2((AccelX - biasAccelX) / 8192, (AccelZ - biasAccelZ) / 8192) * 360.0 / (2*PI);
  rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; 
  rollPrediction = rollPrediction - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
  
  yawGyro = yawGyro - ((gyroResult[2] - biasGyroZ) / 14.375) * timeStep;
}

void kalmanFilter() {
  //-------------Filter Time-----------------KALMAN Filter//
  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
  Pxv += timeStep * Pvv;
  Pxx += timeStep * giroVar;
  Pvv += timeStep * deltaGiroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
  rollPrediction += (rollAccel - rollPrediction) * kx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
}

void derajatServo() {
  pitchServo = pitchPrediction+5;
  rollServo = rollPrediction-5;
    
  pitchServo = constrain(pitchServo, -90, 90);
  pitchServo = map(pitchServo, -90, 90, 0, 180);

  
  rollServo = constrain(rollServo, -90, 90);
  rollServo = map(rollServo, -90, 90, 180, 0);
}

void printData() {
  //Serial.print(AccelX);
  //Serial.print("X \t");
  //Serial.print(AccelY);
  //Serial.print("Y \t");
  //Serial.print(AccelZ);
  //Serial.print("Z \t");
  Serial.print(pitchServo);
  Serial.print("pitch \t"); 
  //Serial.print(Output);
  //Serial.print("roll \t");

  //Serial.print(rollServo);
  //Serial.print("roll \t");
  
  /*
   Serial.print(scaled.XAxis);
   Serial.print("   ");   
   Serial.print(scaled.YAxis);
   Serial.print("   ");   
   Serial.print(scaled.ZAxis);
   
   Serial.print("Heading:\t");
   Serial.print(heading);
   Serial.print(" Radians   \t");
   */
   

   Serial.println(" \n");
}

