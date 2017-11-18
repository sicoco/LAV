#include <Adafruit_NeoPixel.h>
#include <L3G.h>
#include <Wire.h>
#include <Servo.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
#include "MsTimer2.h"
#include "Metro.h"

#define DEBUG


/*
Tasks need to be done at realtime(priority, latency)
motor speed control pid * 2 (HIGH,LOW)
gyro sensor readout and calculation of self orientation(HIGH,LOW)
Infrared distance sensor readout * 3 (MEDIUM,LOW)
urf sensor readout(MEDIUM,HIGH)
servo angle control * 2(LOW,LOW)
RGB LED control(LOW,LOW)
sensor fusion and mapping(HIGH,MEDIUM)
path planning(MEDIUM,HIGH)
serial communication(LOW,MEDIUM)
*/

/*sensor list
 *Backward IR dist sensor A2(ADC)
 *Left IR dist sensor A0(ADC)
 *Right IR dist sensor A1(ADC)
 *URF sensor D7(Echo),D8(Trig)
 *Gyro L3GD20 A4(SDA) A5(SCL)
 */

/*actuator list
 *Left Servo D10(PWM)
 *Right Servo D11(PWM)
 *DRV8835 Dual Motor driver D3(PWM Bin2),D5(PWM Bin1),D6(PWM Ain2),D9(PWM Ain1)
 *Left WS2812 RGB LED D4 #1
 *Right WS2812 RGB LED D4 #0
 */

#define LED_PIN 4
#define R_eye 0
#define L_eye 1
Adafruit_NeoPixel eyes = Adafruit_NeoPixel(2, LED_PIN, NEO_GRB + NEO_KHZ800);

#define L_motor_Pin1 9
#define L_motor_Pin2 6
#define R_motor_Pin1 5
#define R_motor_Pin2 3

#define servoL_Pin 10
#define servoR_Pin 11
Servo servoL, servoR;

#define IRL_Pin A0
#define IRB_Pin A2
#define IRR_Pin A1

#define URF_Echo_Pin 7
#define URF_Trig_Pin 8

unsigned long URF_timeout = 12000;//unit:us

int servoL_angle = 0;//unit:degree
int servoR_angle = 180;//unit:degree

float distF, distL, distR, distB;//unit:cm

int speedL = 0;//PWM
int speedR = 0;//PWM

float depthmap[360];//unit:cm

L3G gyro;
float gyrox,gyroy,gyroz;//under default setting, this data= raw_data*8.75=millidegree per second
float gyrox_bias = 0;
float gyroy_bias = 0;
float gyroz_bias = 0; //unit same as raw data
long gyro_calib_time = 8000;//ms
float gyrox_max = 0;
float gyrox_min = 0;
float gyroy_max = 0;
float gyroy_min = 0;
float gyroz_max = 0;
float gyroz_min = 0;

//static IR ditance sensor calibration data

//sensor position offset data, center point is at the middle point between two wheels' center shaft
float IRL_offsetx = 2.;
float IRL_offsety = 0;
float IRR_offsetx = -2.;
float IRR_offsety = 0;
float IRB_offsetx = 0;
float IRB_offsety = -5;
float URF_offsetx = 0;
float URF_offsety = 4;

Metro URFtask = Metro(6); //6ms interval
Metro IRscantask = Metro(20); //20ms interval
Metro LEDtask = Metro(500); //500ms interval

int robot_state = 0;//it controls the LED's color depend on robot's current state

void setup() {
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("Initializing...");
#endif
  eyes.begin();
  eyes.show(); // Initialize all pixels to 'off'

  delay(2000);
  Wire.begin();
  if (!gyro.init())
  {
#ifdef DEBUG
    Serial.println("Failed to autodetect gyro type!");
#endif
    while (1);
  }
  gyro.enableDefault();
  gyro_calib(gyro_calib_time);

  URF_init();
  servo_init();
  motor_init();

  // MsTimer2::set(5,task);//set interrupt every 5 ms for many tasks like motor control, gyro reading

  // MsTimer2::start(); //allow motor_control() to get started


#ifdef DEBUG
  Serial.println("Setup Complete!");
#endif


}

void loop() {
  // put your main code here, to run repeatedly:

  if(URFtask.check() == 1){
    //URF_scan();
  }

  if(IRscantask.check() == 1){
    //IR_scan();
  }

  if(LEDtask.check() == 1){
    //RGB_task(robot_state);
  }

  //serial_report();  
}

void scan_surrounding(int angleL, int angleR)
{

}

void motor_control(int spL, int spR)
{


}

void navigation()
{

}

void URF_scan()
{
  digitalWrite(URF_Trig_Pin, LOW);
  delayMicroseconds(2);
  digitalWrite(URF_Trig_Pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(URF_Trig_Pin, LOW);
  distF=pulseIn(URF_Echo_Pin,HIGH,URF_timeout)/29/2;//unit:cm
#ifdef DEBUG
  Serial.print("URF: ");
  Serial.println(distF);
#endif
}

void serial_report()
{
#ifdef DEBUG
  Serial.print("X: ");
  Serial.print(gyrox);
  Serial.print(" Y: ");
  Serial.print(gyroy);
  Serial.print(" Z: ");
  Serial.print(gyroz);
  //  Serial.print(" L: ");
  //  Serial.print(distL);
  //  Serial.print(" R: ");
  //  Serial.print(distR);
  //  Serial.print(" B: ");
  //  Serial.print(distB);
  //  Serial.print(" F: ");
  //  Serial.print(distF);
  Serial.println();
#endif
}

void IR_read()
{
  distL=analogRead(IRL_Pin);//every analogRead command takes 102us
  distR=analogRead(IRR_Pin);
  distB=analogRead(IRB_Pin);
}

void IR_scan(){
  servoL.write(servoL_angle);
  servoR.write(servoR_angle);
  IR_read();
#ifdef DEBUG
  Serial.println("IR");
#endif
}
void servo_init()
{
  servoL.attach(servoL_Pin);
  servoR.attach(servoR_Pin);
#ifdef DEBUG
  Serial.println("Servo Init. Done");
#endif
}
void motor_init()
{
  pinMode(L_motor_Pin1,OUTPUT);
  pinMode(L_motor_Pin2,OUTPUT);
  pinMode(R_motor_Pin1,OUTPUT);
  pinMode(R_motor_Pin2,OUTPUT);
  digitalWrite(L_motor_Pin1,LOW);
  digitalWrite(L_motor_Pin2,LOW);
  digitalWrite(R_motor_Pin1,LOW);
  digitalWrite(R_motor_Pin2,LOW);
  speedL = 0;
  speedR = 0;
#ifdef DEBUG
  Serial.println("Motor Init. Done");
#endif
}
void URF_init()
{
  pinMode(URF_Echo_Pin,INPUT);
  pinMode(URF_Trig_Pin,OUTPUT);
#ifdef DEBUG
  Serial.println("URF Init. Done");
#endif
}

void gyro_calib(long waiting)
{
  long count = 0;
#ifdef DEBUG
  Serial.println("Gyro Calibrating... Keep stationary");
#endif

  while(millis()<waiting)
  {
    gyro.read();
    gyrox_bias += gyro.g.x;
    gyroy_bias += gyro.g.y;
    gyroz_bias += gyro.g.z;

    if(gyro.g.x>gyrox_max)
      gyrox_max = gyro.g.x;
    if(gyro.g.y>gyroy_max)
      gyroy_max = gyro.g.y;
    if(gyro.g.z>gyroz_max)
      gyroz_max = gyro.g.z;
    if(gyro.g.x<gyrox_min)
      gyrox_min = gyro.g.x;
    if(gyro.g.y<gyroy_min)
      gyroy_min = gyro.g.y;
    if(gyro.g.z<gyroz_min)
      gyroz_min = gyro.g.z;
    //this process can understand gyro's average value and distribution on each axis at stationary condition
    count++;
    delay(50);
  }
  gyrox_bias /=(float)count;
  gyroy_bias /=(float)count;
  gyroz_bias /=(float)count;
#ifdef DEBUG
  Serial.print("Checked ");
  Serial.print(count);
  Serial.println(" times");
  Serial.print("X average bias: ");
  Serial.println(gyrox_bias);
  Serial.print("Y average bias: ");
  Serial.println(gyroy_bias);
  Serial.print("Z average bias: ");
  Serial.println(gyroz_bias);
  Serial.print("X max: ");
  Serial.println(gyrox_max);
  Serial.print("X min: ");
  Serial.println(gyrox_min);
  Serial.print("Y max: ");
  Serial.println(gyroy_max);
  Serial.print("Y min: ");
  Serial.println(gyroy_min);
  Serial.print("Z max: ");
  Serial.println(gyroz_max);
  Serial.print("Z min: ");
  Serial.println(gyroz_min);
  Serial.println("GYRO CALIBRATION DONE");
#endif
  delay(2000);
}

void RGB_task(int state)
{
  eyes.setPixelColor(R_eye,eyes.Color(random(0,125),random(0,125),random(0,125)));
  eyes.setPixelColor(L_eye,eyes.Color(random(0,125),random(0,125),random(0,125)));
  eyes.show();
#ifdef DEBUG
  Serial.println("LED");
#endif
}

void task()//works every 5 ms, gyro will be read at 200Hz and motor contorl will be done at 100Hz, gyro data can also be used to improve accuracy of IR sensing while robot moving.
{
  static int counter = 0;
  gyro.read();
  gyrox=(float)((int)gyro.g.x-(int)gyrox_bias)*8.75;
  gyroy=(float)((int)gyro.g.y-(int)gyroy_bias)*8.75;
  gyroz=(float)((int)gyro.g.z-(int)gyroz_bias)*8.75;
  counter++;
  if(counter == 2){
    int i,j;
    motor_control(i, j);
    counter = 0; 
  }
}

