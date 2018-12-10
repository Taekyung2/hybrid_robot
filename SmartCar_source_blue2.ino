#include <Wire.h>
#include <TinyGPS++.h>
#include "Wire.h"
#include <math.h>
// I2Cdev and HMC5883L must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "HMC5883L.h"
#include <Servo.h>
#include <SoftwareSerial.h> // 시리얼 통신을 위한 라이브러리 선언

SoftwareSerial BTserial(8, 9); //Tx RX
SoftwareSerial GPSserial(12, 13);

#define PI 3.14159265358979323846264338327950288419


#define Magnetometer_mX0 0x03  
#define Magnetometer_mX1 0x04  
#define Magnetometer_mZ0 0x05  
#define Magnetometer_mZ1 0x06  
#define Magnetometer_mY0 0x07  
#define Magnetometer_mY1 0x08  
#define Magnetometer 0x1E //I2C 7bit address of HMC5883

#define LED_PIN 13
#define BUFF_SIZE 9

#define CAR_DIR_FW  1   // 전진.
#define CAR_DIR_BW  2   // 후진.
#define CAR_DIR_LF  3   // 좌회전.
#define CAR_DIR_RF  4   // 우회전
#define CAR_DIR_ST  5   // 정지.

int mX0, mX1, mX_out;
int mY0, mY1, mY_out;
int mZ0, mZ1, mZ_out;

float heading, headingDegrees, headingFiltered, declination;
float Xm,Ym,Zm;

TinyGPSPlus gps;
HMC5883L mag;

//출력핀(trig)과 입력핀(echo) 설정
int trigPin = 6;                  // 디지털 13번 핀에 연결
int echoPin = 7;                  // 디지털 12번 핀에 연결
long Ulta_d = 0;
long Ultrasonic_sensing();


char BluetoothData;
char GpsData;
int flag=0;
int count = 1;
int cnt=0;
int cnt2=0;
char formation = 'X';
char hive_lat[10] = {'0','0','0','0','0','0','0','0','0'};
char hive_long[10] = {'0','0','0','0','0','0','0','0','0'};
char buf[6] = {'0','0','0','0','0'};
char beacon[9] = {'0','0','0','0','0','0','0','0'};
char hive_compass[4] = {'0','0','0'};
double lat, lng;
int lead_com;
double sub_x, sub_y;
Servo EduServo;

int pos = 0;                      // 서보의 위치
int16_t mx, my, mz;


bool blinkState = false;
uint8_t buffer[BUFF_SIZE];
uint8_t index=0;
uint8_t data;
char makegps[4];
char szRecvCmd;
int flag3=0;
int RightMotor_E_pin = 4;      // 오른쪽 모터의 Enable & PWM
int RightMotor_1_pin = 2;      // 오른쪽 모터 제어선 IN1
int RightMotor_2_pin = 3;     // 오른쪽 모터 제어선 IN2
int LeftMotor_3_pin = 10;      // 왼쪽 모터 제어선 IN3
int LeftMotor_4_pin = 11;      // 왼쪽 모터 제어선 IN4
int LeftMotor_E_pin = 5;      // 왼쪽 모터의 Enable & PWM
int com_count=0,com_flag=0;

int E_carSpeed = 100; // 최대 속도의  60 % 
int prev_speed = 0;

char E_carDirection = 0;
int k=0;
void SmartCar_Go();
void SmartCar_Back();
void SmartCar_Stop();
void SmartCar_Left();
void SmartCar_Right();
void SmartCar_Update();
void controllerByBTCommand(char szBTCmd);

int i=0;

void setup() {
    Serial.begin(9600);
    GPSserial.begin(9600);
    BTserial.begin(9600);
    BTserial.print("AT+DISI?");
    delay(1000);
  
    
  // declare the ledPin as an OUTPUT:  모터 핀설정
    pinMode(RightMotor_E_pin, OUTPUT);        // 출력모드로 설정
    pinMode(RightMotor_1_pin, OUTPUT);
    pinMode(RightMotor_2_pin, OUTPUT);
    pinMode(LeftMotor_3_pin, OUTPUT);
    pinMode(LeftMotor_4_pin, OUTPUT);
    pinMode(LeftMotor_E_pin, OUTPUT);

    
    digitalWrite(RightMotor_E_pin, HIGH);     // 오른쪽 모터의 Enable 핀 활성화
    digitalWrite(LeftMotor_E_pin, HIGH);      // 왼쪽 모터의 Enable 핀 활성화
    
   
 pinMode(echoPin, INPUT);   // echoPin 입력
  pinMode(trigPin, OUTPUT);  // trigPin 출력
   
    Wire.begin();
    delay(100);

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    // initialize device
    
   
    mag.initialize();

  
    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);  
    Wire.beginTransmission(Magnetometer); 
    Wire.write(0x02); // Select mode register
    Wire.write(0x00); // Continuous measurement mode 
    Wire.endTransmission();
}

void loop() {
       
    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
    double result_a,result_b,radian;
    int sub_compass;
/*
 while(k++<15) sub_compass = getCompass();

    if(i++%30 == 0) {
        sub_compass = getCompass();
      //  Serial.println(sub_compass);
  
    }
    */

 
  
  
        if(flag==0) {
    
              BTserial.listen();
         
                while(BTserial.available()>0) {    
                    
                    for(int a=4; a>0; a--){
                        buf[4-a] = buf[5-a]; 
                    }
                    for(int b=7; b>0; b--){
                        beacon[7-b] = beacon[8-b];  
                    }
                    buf[4] = BTserial.read();
                    Serial.print(buf[4]);
                    beacon[7] = buf[4];
                  
                    if(beacon[0] == '0' && beacon[1] == '0' && beacon[2] == '2' && beacon[3] == '1' && beacon[4] == '5'
                    && beacon[5] == ':' && beacon[6] == '0'  ){
                        count = -22;
                      
                    }
                    if(count==-22){
                        formation = beacon[7];          
                    }
                    else if(-22<count && count<-12){
                        hive_lat[count+21] = beacon[7];                
                        if(count == -13)
                            lat=atof(hive_lat)/1000000;
                    }
                    else if(-13<count && count<-3){
                        hive_long[count+12] = beacon[7];
                        if(count == -4)                  
                            lng=atof(hive_long)/1000000;     
                    }
                    else if(count>-4 && count <0) {
                        hive_compass[count+3] = beacon[7];
                        if(count==-1) 
                            lead_com= atoi(hive_compass);
                    
                    }
                    count++;
                    
                    if(buf[0] == 'D' && buf[1] == 'I' && buf[2] == 'S' && buf[3] == 'C' && buf[4] == 'E'){
                        BTserial.print("AT+DISI?");
                        
                        delay(100);
                             Serial.println();    
                        buf[0] = '0';
                        buf[1] = '0';
                        buf[2] = '0';
                        buf[3] = '0';
                        buf[4] = '0';
                        
                        /*flag=1;
                
                         
                        Serial.println(); Serial.print("formation = ");Serial.print(formation); Serial.print("  "); Serial.print("lat = "); Serial.print(lat); 
                         Serial.print("  ");  Serial.print("lng  = "); Serial.print(lng);  Serial.print("  ");  Serial.println(lead_com);     Serial.println();
              */
                    }

                }
         
         }
             


        if(flag==1) {
            GPSserial.listen();
           delay(20);
            while(GPSserial.available()>0){   
                Serial.println();
                sub_x= printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
                sub_y=printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);  
                printDateTime(gps.date, gps.time);
                Serial.println();
                smartDelay(1000);
             
                if (millis() > 5000 && gps.charsProcessed() < 10) 
                    Serial.println(F("No GPS data received: check wiring"));
                
                flag=0;
           
                delay(20);
                break;
                    
            }
       
        }

      
  
        if(formation == 'A') {
           formation_A(lat,lng,lead_com,sub_compass,sub_x,sub_y,i);  // 인자로 리드의 gps값, compass
        }
        
        
        if(formation =='B') {
           formation_B(lat,lng,lead_com,sub_compass,sub_x,sub_y);  // 인자로 리드의 gps값, compass
        }
        if(formation == 'C') {
         formation_C(lat,lng,lead_com,sub_compass,sub_x,sub_y);  // 인자로 리드의 gps값, compass
        }
      
       delay(30);


}

     
// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
unsigned long start = millis();
    do 
    {
        while (GPSserial.available())      
              gps.encode(GPSserial.read());
    } while (millis() - start < ms);
}

static double printFloat(float val, bool valid, int len, int prec)
{
    if (!valid)
    {
        while (len-- > 1)
            Serial.print('*');
        Serial.print(' ');
    }
    else
    {
        Serial.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1); // . and -
                     flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;    
        for (int i=flen; i<len; ++i)
            Serial.print(' ');
    }
    
    smartDelay(0);
    
    return val;
}

static void printInt(unsigned long val, bool valid, int len)
{
    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i=strlen(sz); i<len; ++i)
        sz[i] = ' ';
    if (len > 0) 
        sz[len-1] = ' ';
    Serial.print(sz);
    
    smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
    if (!d.isValid())
    {
        Serial.print(F("********** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        Serial.print(sz);
    }
    
    if (!t.isValid())
    {
        Serial.print(F("******** "));
    }
    else
    {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        Serial.print(sz);
    }

    printInt(d.age(), d.isValid(), 5);
    smartDelay(0);
}

// lead 정보로 목표 좌표를 구하고 sub를 이용해 각도를 틀어 움직인다.
int formation_A(double lead_x, double lead_y,int lead_compass,double sub_compass,double sub_x, double sub_y, int i) {  //x = 위도(세로)  y= 경도(가로)
    double a,b, x,y;
    double radian,distance;
    int flag =0,j=0;
    double cal_com;
    double result_a,result_b;
    
      lead_compass =   lead_compass - 90;  
        if(  lead_compass <0)   lead_compass =  lead_compass +360;
        
    while(j++<5) getCompass();
  
     lead_x*=(110.979309*1000);   
     lead_y*=(88.907949*1000);
    
     
    //int d=0;   이건 임의로 포메이션 대열에서 개체간 거리 조절용  
    radian = getRadian(lead_compass);
    a = lead_x - (5.0*cos(radian));   //5대신 d를 넣어서 이용가능
    b = lead_y - (5.0*sin(radian));
/*
    a = lead_x - (10*cos(radian));   //5대신 d를 넣어서 이용가능
    b = lead_y - (10*sin(radian));
    /*
    a = lead_x - (15*cos(radian));   //5대신 d를 넣어서 이용가능
    b = lead_y - (15*sin(radian));
*/

     result_a=a/(110.979309*1000);
    result_b=b/(88.907949*1000);

     cal_com = atan2(a-(sub_x*110.979309*1000 ), b-(sub_y*88.907949*1000));
     cal_com = cal_com*180.0/PI;

    if(cal_com<0) 
        cal_com +=360;
    
    adjust_compass(cal_com);
       
     Ultrasonic_sensing();
     
    delay(100);

}

void formation_B(double lead_x, double lead_y,int lead_compass,int sub_compass, double sub_x, double sub_y){
    double a,b, x,y;
    double radian,distance;
    int theta=45,j=0;
    double cal_com;
    double result_a,result_b;
 lead_compass =   lead_compass - 90;  
        if(  lead_compass <0)   lead_compass =  lead_compass +360;
        
    while(j++<5) getCompass();

     lead_x*=(110.979309*1000);   
     lead_y*=(88.907949*1000);
     
    //int d=0;   이건 임의로 포메이션 대열에서 개체간 거리 조절용  
    radian = getRadian(lead_compass);
    a = lead_x - (5.0*cos(radian));   //5대신 d를 넣어서 이용가능
    b = lead_y - (5.0*sin(radian));
    /* 
       radian= getRadian(theta);
     //대각선 뒤에 있는 놈들의 경우
        a = lead_x - (5*sqrt(2.0)*cos(radian));   
        b = lead_y - (5*sqrt(2.0)*sin(radian));
     
    
    /*    radian= getRadian(theta);
      //대각선 뒤에 있는 놈들의 경우
        a = lead_x + (5*sqrt(2.0)*cos(radian));   
        b = lead_y + (5*sqrt(2.0)*sin(radian));
        
     */
     
    result_a=a/(110.979309*1000);
    result_b=b/(88.907949*1000);

    cal_com = atan2(a-(sub_x*110.979309*1000 ), b-(sub_y*88.907949*1000));
    cal_com = cal_com*180.0/PI;
    
   if(cal_com<0) 
        cal_com +=360;
    
  
    adjust_compass(cal_com);
    Ultrasonic_sensing();
  
  delay(100);
}

void formation_C(double lead_x, double lead_y,int lead_compass,int sub_compass, double sub_x, double sub_y) {
    double a,b, x,y;
    double radian,distance;
    int cal_com,j=0;
    double result_a,result_b;
    int theta = 45;
 lead_compass =   lead_compass - 90;  
        if(  lead_compass <0)   lead_compass =  lead_compass +360;
        

    while(j++<5) getCompass();
     
     lead_x*=(110.979309*1000);
     lead_y*=(88.907949*1000);
     
  
    radian = getRadian(lead_compass);  // 그냥 뒤에 있는 놈의 경우
    a = lead_x - (10*cos(radian));   //5대신 d를 넣어서 이용가능
    b = lead_y - (10*sin(radian));
  
    /*
    radian= getRadian(theta);
     //대각선 뒤에 있는 놈들의 경우
        a = lead_x - (5*sqrt(2.0)*cos(radian));   
        b = lead_y - (5*sqrt(2.0)*sin(radian));

    
     
    /*    radian= getRadian(theta);
      //대각선 뒤에 있는 놈들의 경우
        a = lead_x + (5*sqrt(2.0)*cos(radian));   
        b = lead_y + (5*sqrt(2.0)*sin(radian));
     */

    result_a=a/(110.979309*1000);
    result_b=b/(88.907949*1000);

     //cal_com = cal_compass(lead_x,lead_y, sub_x, sub_y);
     cal_com = atan2(a-(sub_x*110.979309*1000 ), b-(sub_y*88.907949*1000));
     cal_com = cal_com*180.0/PI;
     
     if(cal_com<0) 
          cal_com +=360;
    
    adjust_compass(cal_com);
    Ultrasonic_sensing();

    delay(100);
}

int cal_compass(double goal_x, double goal_y, double x, double y){
    int slope,result;
    double theta;

    theta = atan2(goal_y-y,goal_x-x);  // 기울기로 세타값을 구한다.
    result = getRadian(theta);
    return result;
}

int adjust_compass(int cal_compass) {
    int temp = abs(cal_compass - getCompass());
    int result;
    result=temp;
   /*
    Serial.println(result);
    Serial.println(cal_compass);
    Serial.println(getCompass()); */
    
    if(result>20 ) {
        result=abs(cal_compass - getCompass());
       /* Serial.print("result:  "); Serial.print(result); Serial.print(" cal_compass : "); Serial.print(cal_compass); Serial.print("  getCompass : "); Serial.println(getCompass());
        Serial.println(cal_compass);
*/
        if(result <= 180) {
            SmartCar_Right();          
            delay(100);  
        }
        else if(result > 180) {
             SmartCar_Left(); //Rotate_L        
            delay(100);
        }
    }  
    else  {
        SmartCar_Go();
        delay(100);
    }
}

int getRadian(int num) {
    return int(num*(PI/180.0));
}


void speed_up() {
    prev_speed = E_carSpeed;
    E_carSpeed += 20;
    E_carSpeed = min(E_carSpeed, 255);
}

void speed_down() {
    prev_speed = E_carSpeed;
    E_carSpeed -= 20;
    E_carSpeed = max(E_carSpeed, 50);
}
void SmartCar_Update()
{
    if(E_carDirection == CAR_DIR_FW)  // 전진
        SmartCar_Go();
    else if(E_carDirection ==               CAR_DIR_BW) // 후진
        SmartCar_Back();
    else if(E_carDirection == CAR_DIR_LF) // 좌회전
        SmartCar_Left();
    else if(E_carDirection == CAR_DIR_RF) // 우회전
        SmartCar_Right();
    else if(E_carDirection == CAR_DIR_ST) // 정지.
    
    SmartCar_Stop();
}

double Calc(double Lat1, double Long1, double Lat2, double Long2)
{
/*
The Haversine formula according to Dr. Math.
http://mathforum.org/library/drmath/view/51879.html

*/
    double dDistance;
    const double kEarthRadiusKms = 6376.5; 
    double dLat1InRad = Lat1 * (PI / 180.0);
    double dLong1InRad = Long1 * (PI / 180.0);
    double dLat2InRad = Lat2 * (PI / 180.0);
    double dLong2InRad = Long2 * (PI / 180.0);

    double dLongitude = dLong2InRad - dLong1InRad;
    double dLatitude = dLat2InRad - dLat1InRad;

    // Intermediate result a.
        double a = pow(sin(dLatitude / 2.0), 2.0) + 
        cos(dLat1InRad) * cos(dLat2InRad) * 
        pow(sin(dLongitude / 2.0), 2.0);

    // Intermediate result c (great circle distance in Radians).
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    // Distance.
    dDistance = kEarthRadiusKms * c;

    return dDistance*1000;
}

double Calc(char NS1, double Lat1, double Lat1Min, 
    char EW1, double Long1, double Long1Min, char NS2, 
    double Lat2, double Lat2Min, char EW2, 
    double Long2, double Long2Min)
{
    double NS1Sign = (NS1 == 'N') ? 1.0 : -1.0;
    double EW1Sign = (NS1 == 'E') ? 1.0 : -1.0;
    double NS2Sign = (NS2 == 'N') ? 1.0 : -1.0;
    double EW2Sign = (EW2 == 'E') ? 1.0 : -1.0;
    return (Calc(
        (Lat1 + (Lat1Min / 60)) * NS1Sign,
        (Long1 + (Long1Min / 60)) * EW1Sign,
        (Lat2 + (Lat2Min / 60)) * NS2Sign,
        (Long2 + (Long2Min / 60)) * EW2Sign
        ));
}

int getCompass(void) {
    int x;
    //---- X-Axis
    Wire.beginTransmission(Magnetometer); // transmit to device
    Wire.write(Magnetometer_mX1);
    Wire.endTransmission();
    Wire.requestFrom(Magnetometer,1); 
    if(Wire.available()<=1)   
    {
        mX0 = Wire.read();
    }
    Wire.beginTransmission(Magnetometer); // transmit to device
    Wire.write(Magnetometer_mX0);
    Wire.endTransmission();
    Wire.requestFrom(Magnetometer,1); 
    if(Wire.available()<=1)   
    {
        mX1 = Wire.read();
    }
    
    //---- Y-Axis
    Wire.beginTransmission(Magnetometer); // transmit to device
    Wire.write(Magnetometer_mY1);
    Wire.endTransmission();
    Wire.requestFrom(Magnetometer,1); 
    if(Wire.available()<=1)   
    {
        mY0 = Wire.read();
    }
    Wire.beginTransmission(Magnetometer); // transmit to device
    Wire.write(Magnetometer_mY0);
    Wire.endTransmission();
    Wire.requestFrom(Magnetometer,1); 
    if(Wire.available()<=1)   
    {
        mY1 = Wire.read();
    }
    
    //---- Z-Axis
    Wire.beginTransmission(Magnetometer); // transmit to device
    Wire.write(Magnetometer_mZ1);
    Wire.endTransmission();
    Wire.requestFrom(Magnetometer,1); 
    if(Wire.available()<=1)   
    {
        mZ0 = Wire.read();
    }
    Wire.beginTransmission(Magnetometer); // transmit to device
    Wire.write(Magnetometer_mZ0);
    Wire.endTransmission();
    Wire.requestFrom(Magnetometer,1); 
    if(Wire.available()<=1)   
    {
        mZ1 = Wire.read();
    }
    
    //---- X-Axis
    mX1=mX1<<8;
    mX_out =mX0+mX1; // Raw data
    // From the datasheet: 0.92 mG/digit
    Xm = mX_out*0.00092; // Gauss unit
    
    //---- Y-Axis
    mY1=mY1<<8;
    mY_out =mY0+mY1;
    Ym = mY_out*0.00092;
    
    //---- Z-Axis
    mZ1=mZ1<<8;
    mZ_out =mZ0+mZ1;
    Zm = mZ_out*0.00092;
    
    //Calculating Heading
    heading = atan2(Ym, Xm);

    declination = 0.073; 
    heading += declination;

    if(heading <0) heading += 2*PI; // Correcting when signs are reveresed
    
    if(heading > 2*PI)heading -= 2*PI;  // Correcting due to the addition of the declination angle
    
    headingDegrees = heading * 180/PI; // The heading in Degrees unit
    
    headingFiltered = headingFiltered*0.85 + headingDegrees*0.15; // Smoothing the output angle / Low pass filter 
    
    //Sending the heading value through the Serial Port to Processing IDE
 
    
    x= int(headingFiltered) - 90;
    
    if(x<0) x=x+360;
    return x;
}

long Ultrasonic_sensing()
{
    long duration, distance;
    digitalWrite(trigPin, HIGH);  // trigPin에서 초음파 발생(echoPin도 HIGH)
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

 
    duration = pulseIn(echoPin, HIGH);    // echoPin 이 HIGH를 유지한 시간을 저장 한다.
    distance = ((float)(340 * duration) / 1000) / 2; 
        
    Serial.println("\nDIstance:"); // 물체와 초음파 센서간 거리를 표시
    Serial.println(distance);

     if(distance < 100) {         //장애물 감지 (20cm 이내)
         Serial.println("stop");
         SmartCar_Stop();  //정지(1초)
         delay(1000); 
     }
    return distance;
}
  
void SmartCar_Go() 
// 스마트카 동작 함수들void SmartCar_Go()  // 전진
{
    digitalWrite(RightMotor_1_pin, HIGH);    
    digitalWrite(RightMotor_2_pin, LOW);
    digitalWrite(LeftMotor_3_pin, HIGH);    
    digitalWrite(LeftMotor_4_pin, LOW);

    for(int i=prev_speed; i<=E_carSpeed; i=i+5){ 
      analogWrite(RightMotor_E_pin, i*1.3); 
      analogWrite(LeftMotor_E_pin, i*1.3);
      delay(20); 
    }
   prev_speed = E_carSpeed;

}


void SmartCar_Back() // 후진
{
    digitalWrite(RightMotor_1_pin, LOW);    
    digitalWrite(RightMotor_2_pin, HIGH);
    digitalWrite(LeftMotor_3_pin, LOW);    
    digitalWrite(LeftMotor_4_pin, HIGH);

    for(int i=prev_speed; i<=E_carSpeed; i=i+5){
      analogWrite(RightMotor_E_pin, i); 
      analogWrite(LeftMotor_E_pin, i);    
      delay(20); 
    }
    prev_speed = E_carSpeed;
}

void SmartCar_Left()  // 좌회전
{
    digitalWrite(RightMotor_1_pin, HIGH);    
    digitalWrite(RightMotor_2_pin, LOW);
    digitalWrite(LeftMotor_3_pin, HIGH);    
    digitalWrite(LeftMotor_4_pin, LOW);
        
    for(int i=prev_speed; i<=E_carSpeed; i=i+5){
      analogWrite(RightMotor_E_pin, i*1.4);             // 140%
      analogWrite(LeftMotor_E_pin, i*0.2);              // 20%
      delay(50); 
    }
    prev_speed = E_carSpeed;
}

void SmartCar_Right() // 우회전
{
    digitalWrite(RightMotor_1_pin, HIGH);    
    digitalWrite(RightMotor_2_pin, LOW);
    digitalWrite(LeftMotor_3_pin, HIGH);    
    digitalWrite(LeftMotor_4_pin, LOW);
    
    for(int i=prev_speed; i<=E_carSpeed; i=i+5){
      analogWrite(RightMotor_E_pin, i*0.2);             // 20%
      analogWrite(LeftMotor_E_pin, i*1.4);              // 140%
      delay(50); 
    }  
    prev_speed = E_carSpeed;
}

void SmartCar_Stop()  // 정지
{
    if(E_carDirection == CAR_DIR_FW || E_carDirection == CAR_DIR_LF || E_carDirection == CAR_DIR_RF){
      for(int i=E_carSpeed; i>=0; i=i-5){
        analogWrite(RightMotor_E_pin, i);  
        analogWrite(LeftMotor_E_pin, i);
        delay(20); 
      }
    }else if(E_carDirection == CAR_DIR_BW){
      for(int i=E_carSpeed; i>=0; i=i-5){
        analogWrite(RightMotor_E_pin, i);  
        analogWrite(LeftMotor_E_pin, i);
        delay(20); 
      }
    }
    digitalWrite(RightMotor_E_pin, LOW); // 정지
    digitalWrite(LeftMotor_E_pin, LOW);
}

void controllerByBTCommand(char szBTCmd)
{
    if (szBTCmd == '+') // 모터의 speed up
    {
        prev_speed = E_carSpeed;
        E_carSpeed += 20;
        E_carSpeed = min(E_carSpeed, 255);
        Serial.print("Speed Up ");
        Serial.println(E_carSpeed);
    }
    else
    if (szBTCmd == '-') // 모터의 speed down
    {
        prev_speed = E_carSpeed;
        E_carSpeed -= 20;
        E_carSpeed = max(E_carSpeed, 50);
        Serial.print("Speed down ");
        Serial.println(E_carSpeed);
    }
    else if(szBTCmd == 'g') // 전진
        E_carDirection = CAR_DIR_FW;  
    else if(szBTCmd == 's') // 정지
        E_carDirection = CAR_DIR_ST;  
    else if(szBTCmd == 'b') // 후진
        E_carDirection = CAR_DIR_BW;    
    else if(szBTCmd == 'l')  // 좌회전
        E_carDirection = CAR_DIR_LF;    
    else if(szBTCmd == 'r')  // 우회전
        E_carDirection = CAR_DIR_RF;   
}

