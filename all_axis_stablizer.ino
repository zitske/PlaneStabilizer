#include<Wire.h>
#include<Servo.h>
#include <SD.h>

#define battPin0 A11    // select the input pins for the batteries
#define battPin1 A14
#define battPin2 A12

Servo servo;
Servo servo2;
Servo servo3;


int servo_pin = A3;
int servo_pin2 = A2;
int servo_pin3 = A1;
int sd_cs_pin = 53;
int light_pin = A6;
unsigned long t = 0;


//Control surfaces phisical limits
int p_airelon_max = 150; // Airelon
int p_airelon_min = 30; // Airelon
int p_flap_r_max = 100; // Flap
int p_flap_r_min = 0; // Flap
int p_flap_l_max = 80; // Flap
int p_flap_l_min = 180; // Flap
int p_rudder_max = 180; // Rudder
int p_rudder_min = 0; // Rudder
int p_elevator_max = 120; // Elevator
int p_elevator_min = 30; // Elevator

File file;

// Start MPU variables
const int MPU_addr = 0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int Yangle,Xangle,Zangle;
// End MPU variables

// Start battery variables
float battWeighting = 0.1; // define a weighting to apply to our exponential moving average calculation for battery 

int abattValue0 = 0; // variable to store average value (exponential moving average) calculation
int abattValue1 = 0;
int abattValue2 = 0;

float Cell1 = 0.00; // variable to store actual cell voltages
float Cell2 = 0.00;
float Cell3 = 0.00;

float percentage;
float adcVolt = 0.0041780351906158 ; // one point on the ADC equals this many volts
// End battery variables

void setup() {
  pinMode(light_pin,OUTPUT);
  
  Serial.begin(115200);
  
  mpu_setup();
  
  //Starting SD
  SD.begin(sd_cs_pin);

  servos_attach();
  get_baterry_average(50);
  
}

void loop() {
  get_GyroData();
  update_Servo_position();
  write_log(generate_logData());
  update_time();
  delay(20);
}
float getBattVolts() {
  // read the value from the sensor:
  abattValue0 = (analogRead(battPin0) * battWeighting) + (abattValue0 * (1-battWeighting));
  abattValue1 = (analogRead(battPin1) * battWeighting) + (abattValue1 * (1-battWeighting));
  abattValue2 = (analogRead(battPin2) * battWeighting) + (abattValue2 * (1-battWeighting));
  //abattValue3 = (analogRead(battPin3) * battWeighting) + (abattValue3 * (1-battWeighting));
  // convert these values to cell voltages
  Cell1 = (adcVolt * abattValue0 * 1) ;
  Cell2 = (adcVolt * abattValue1 * 1.7298)-Cell1;
  Cell3 = (adcVolt * abattValue2 * 2.6078)-Cell2-Cell1;
  //Cell4 = (adcVolt * abattValue3 * 3.7659)-Cell3-Cell2-Cell1;
  return Cell1+Cell2+Cell3;
}

void showBattVolts() {
  Serial.print (Cell1);
  Serial.print ("V. " );
  Serial.print (Cell2);
  Serial.print ("V. ");
  Serial.print (Cell3);
  Serial.print ("V. Total = " );
  Serial.print (Cell1+Cell2+Cell3);
  Serial.print (" " );
  Serial.print(percentage*100);
  Serial.println("%");
}

float get_percentage(float volt){
    float low  = 3.0 * 3;
    float high = 4.2 * 3;
    percentage = constrain((volt - low) / (high - low), 0.0, 1.0);
    return percentage*100;
}

void get_GyroData(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  Yangle = AcY/182.04;
  Xangle = AcX/182.04;
  Zangle = GyZ/182.04;
}

void update_time(){
  t++;
}

String generate_logData(){
  String dataString ="";
  dataString += String(t);
  dataString += String(",");
  dataString += String(Xangle);
  dataString += String(",");
  dataString += String(constrain(Xangle + 90,p_airelon_min,p_airelon_max));
  dataString += String(",");
  dataString += String(Yangle);
  dataString += String(",");
  dataString += String(constrain(-Yangle + 90,p_elevator_min,p_elevator_max));
  dataString += String(",");
  dataString += String(Zangle);
  dataString += String(",");
  dataString += String(constrain(Zangle + 90,p_rudder_min,p_rudder_max));
  dataString += String(",");
  dataString += String(int(get_percentage(getBattVolts())));
  dataString += String(",");
  dataString += String(int(getBattVolts()));
  return dataString;
}

void write_log(String data){
  file = SD.open("log.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (file) {
    file.println(data);
    file.close();
  }
}

void update_Servo_position(){
  servo.write(constrain(Zangle+90,p_rudder_min,p_rudder_max));
  servo2.write(constrain(Xangle + 90,p_airelon_min,p_airelon_max));
  servo3.write(constrain(-Yangle + 90,p_elevator_min,p_elevator_max));
}

void light_mode(int mode){
  switch(mode){
    case 0:
    digitalWrite(light_pin,LOW);
    break;
    case 1:
    digitalWrite(light_pin,HIGH);
    break;
    default:
    digitalWrite(light_pin,LOW);
    break;
  }
}

void mpu_setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void servos_attach(){
  servo.attach(servo_pin);
  servo2.attach(servo_pin2);
  servo3.attach(servo_pin3);
}

void get_baterry_average(int times){
  for (int i=0; i<times; i++){
      getBattVolts();
    }
}