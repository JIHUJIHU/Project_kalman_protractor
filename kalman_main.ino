#include <kalman.h>
#include <Wire.h>
#include "Kalman.h"

Kalman kalmanX; 
Kalman kalmanY;

const uint8_t IMUAddress = 0x68;
const uint16_t I2C_TIMEOUT = 1000; 

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop);
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t* data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  return Wire.endTransmission(sendStop);
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t* data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  if (Wire.endTransmission(false)) 
    return 1; 
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else
        return 2;
    }
  }
  return 0;
}


int16_t accX, accY, accZ;
int16_t tempRaw;
int16_t gyroX, gyroY, gyroZ;

int accXangle, accYangle;
int temp; // Temperature
int gyroXangle, gyroYangle; 
int compAngleX, compAngleY; 
int kalAngleX, kalAngleY; 

uint32_t timer;
uint8_t i2cData[14]; 



#define LED 13
#define ANALOG A0
int A0_Value = 0;
int su, su2, su3, su4;
int segment_data;


void setup() {                                 //말그대로 셋업...
  pinMode(13, INPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode( A2, INPUT_PULLUP);
  pinMode (A0, INPUT_PULLUP);
  Serial.begin(115200);


  digitalWrite(5, 0);
  digitalWrite(4, 0);
  digitalWrite(3, 0);

  segment_data = 987;

  Wire.begin();
  i2cData[0] = 7; 
  i2cData[1] = 0x00;
  i2cData[2] = 0x00; 
  i2cData[3] = 0x00; 
  while (i2cWrite(0x19, i2cData, 4, false)); 
  while (i2cWrite(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { 
    Serial.print(F("Error reading sensor"));
    while (1);

  }

  delay(100);

  while (i2cRead(0x3B, i2cData, 6));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;

  kalmanX.setAngle(accXangle); 
  kalmanY.setAngle(accYangle);
  gyroXangle = accXangle;
  gyroYangle = accYangle;
  compAngleX = accXangle;
  compAngleY = accYangle;

  timer = micros();

}



int flag = 0;                                 //flag 변수를 전역변수로 초기화
int first=0, second=0, final=0;                           //first, second, final변수도 전역변수로 초기화
void loop() {
  int i;                              
  int button;                                 //button 변수 - 버튼의 입력여부 파악
  
  


  angleValue();                                  //감지 함수

  Serial.print(kalAngleX);                           //x 값 프린트
  Serial.print('\t');   
  Serial.print(kalAngleY);                           //y 값 프린트
  Serial.println('\t');
  if (digitalRead(A0) == 1) {  //키가 안 눌렸을때는 각도값을 세그먼트변수에 넣는디.....
    segment_data = kalAngleY;
  }
  button=analogRead(2);                              //button 변수에 버튼의 입력여부를 저장
  Serial.println(button);                           //button 입력여부를 시리얼 모니터에 출력
  displaySegment();                              //버튼의 입력이 들어오기 전엔 현재 값을 끊임 없이 프린트
  
  if(button<=500&&flag==1){                           //버튼이 두번째로 입력되었고(flag변수는 첫번째인지 두번째로 눌렀는지 파악위함)
    second=segment_data;                           //second 변수에 현재 값을 저장
    Serial.print("Second : ");                           //second 변수에 저장된 값을 출력한다
    Serial.println(second);
    flag =0;                                 //flag변수를 초기화
     if(second>=first){                              //first, second 변수 중 큰수에서 작은수를뺌. 두 수의 차이를 구함.
      final = second - first;                           //final 변수에 결과 값을 저장한다.
    }else{
      final= first - second;
    }
    for(int i=0;i<100;i++){                           //그냥 delay를 주면 값이 꺼져버려서,(이유는 displaySegment의 형태 확인)
     displaySegment();                              //for문으로 끊임없이 display에 출력하면서 딜레이를 줌.
       delay(1);                              // - 두번째 값 홀딩
     }
    segment_data=final;                              //결과 값을 출력하기 위해 현재 값을 결과 값으로 지정.
    
  }
  else if(button<=500&&flag==0){                        //버튼이 첫번째로 입력되었고(flag변수는 첫번째인지 두번째로 눌렀는지 파악위함)
    first=segment_data;                              //first 변수에 현재 값을 저장
    Serial.print("first : ");                           //first 변수에 저장된 값을 출력한다.
    Serial.println(first);                              
    flag=1;                                 //flag변수를 1로 줌으로서 다음 버튼입력은 두번째임을 명시.
    for(int i=0;i<1000;i++){                           //그냥 delay를 주면 값이 꺼져버려서,(이유는 displaySegment의 형태 확인)
     displaySegment();                              //for문으로 끊임없이 display에 출력하면서 딜레이를 줌.
       delay(1);                              // - 첫번째 값 홀딩
     }
  }
  if(final!=0){                                 //결과 값 저장된 것이 있다면,
    Serial.print("Final : ");                           //final 변수에 저장된 값을 출력한다.
    Serial.println(final);
    
      for(int i=0;i<1000;i++){                           //그냥 delay를 주면 값이 꺼져버려서,(이유는 displaySegment의 형태 확인)
     displaySegment();                              //for문으로 끊임없이 display에 출력하면서 딜레이를 줌
       delay(1);                              // - 결과 값 홀딩
     }
    first=second=final=0;                           //first, second, first 모든 값을 0으로 초기화
  }
}

void displaySegment() {                              //현재값(segment_data)를 출력해주는 함수
  int d3, d2, d1;

 
  d3 = segment_data / 100;
  d2 = segment_data % 100 / 10;
  d1 = segment_data % 10;

  digitalWrite(5, 0);
  digitalWrite(4, 0);
  digitalWrite(3, 0);

  what(d3);
  digitalWrite(5, 1);
  delay(2);

  digitalWrite(5, 0);
  what(d2);
  digitalWrite(4, 1);
  delay(2);

  digitalWrite(4, 0);
  what(d1);
  digitalWrite(3, 1);
  delay(2);
}



void angleValue() {                              //현재값을 계산하는 함수
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = ((i2cData[6] << 8) | i2cData[7]);
  gyroX = ((i2cData[8] << 8) | i2cData[9]);
  gyroY = ((i2cData[10] << 8) | i2cData[11]);
  gyroZ = ((i2cData[12] << 8) | i2cData[13]);

 
  accXangle = (atan2(accY, accZ) + PI) * RAD_TO_DEG;
  accYangle = (atan2(accX, accZ) + PI) * RAD_TO_DEG;

  double gyroXrate = (double)gyroX / 131.0;
  double gyroYrate = -((double)gyroY / 131.0);
  gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); 
  gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000);


  compAngleX = (0.93 * (compAngleX + (gyroXrate * (double)(micros() - timer) / 1000000))) + (0.07 * accXangle); 
  compAngleY = (0.93 * (compAngleY + (gyroYrate * (double)(micros() - timer) / 1000000))) + (0.07 * accYangle);

  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros() - timer) / 1000000); 
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros() - timer) / 1000000);
  timer = micros();

  temp = ((double)tempRaw + 12412.0) / 340.0;


}




void what(int a) {                              //fnd 세자리 각각을 출력하기 위한 함수(숫자에 따라)
  switch (a) {
    case 1 : one();
      break;
    case 2 : two();
      break;
    case 3 : three();
      break;
    case 4 : four();
      break;
    case 5 : five();
      break;
    case 6 : six();
      break;
    case 7 : seven();
      break;
    case 8 : eight();
      break;
    case 9 : nine();
      break;
    case 0 : zero();
      break;
    default : zero();
      break;
  }
}

void zero() {
  digitalWrite(12, 1);
  digitalWrite(11, 0);
  digitalWrite(10, 0);
  digitalWrite(9, 0);
  digitalWrite(8, 0);
  digitalWrite(7, 0);
  digitalWrite(6, 0);
}

void one() {
  digitalWrite(12, 1);
  digitalWrite(11, 0);
  digitalWrite(10, 0);
  digitalWrite(9, 1);
  digitalWrite(8, 1);
  digitalWrite(7, 1);
  digitalWrite(6, 1);
}

void two() {
  digitalWrite(12, 0);
  digitalWrite(11, 1);
  digitalWrite(10, 0);
  digitalWrite(9, 0);
  digitalWrite(8, 1);
  digitalWrite(7, 0);
  digitalWrite(6, 0);
}

void three() {
  digitalWrite(12, 0);
  digitalWrite(11, 1);
  digitalWrite(10, 1);
  digitalWrite(9, 0);
  digitalWrite(8, 0);
  digitalWrite(7, 0);
  digitalWrite(6, 0);
}

void four() {
  digitalWrite(12, 0);
  digitalWrite(11, 0);
  digitalWrite(10, 1);
  digitalWrite(9, 1);
  digitalWrite(8, 0);
  digitalWrite(7, 0);
  digitalWrite(6, 1);
}

void five() {
  digitalWrite(12, 0);
  digitalWrite(11, 0);
  digitalWrite(10, 1);
  digitalWrite(9, 0);
  digitalWrite(8, 0);
  digitalWrite(7, 1);
  digitalWrite(6, 0);
}

void six() {
  digitalWrite(12, 0);
  digitalWrite(11, 0);
  digitalWrite(10, 0);
  digitalWrite(9, 0);
  digitalWrite(8, 0);
  digitalWrite(7, 1);
  digitalWrite(6, 1);
}

void seven() {
  digitalWrite(12, 1);
  digitalWrite(11, 1);
  digitalWrite(10, 1);
  digitalWrite(9, 1);
  digitalWrite(8, 0);
  digitalWrite(7, 0);
  digitalWrite(6, 0);
}

void eight(){
  digitalWrite(12, 0);
  digitalWrite(11, 0);
  digitalWrite(10, 0);
  digitalWrite(9, 0);
  digitalWrite(8, 0);
  digitalWrite(7, 0);
  digitalWrite(6, 0);
}

void nine() {
  digitalWrite(12, 0);
  digitalWrite(11, 0);
  digitalWrite(10, 1);
  digitalWrite(9, 1);
  digitalWrite(8, 0);
  digitalWrite(7, 0);
  digitalWrite(6, 0);
}
