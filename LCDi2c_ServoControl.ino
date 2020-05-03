#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include<Servo.h>
#include <avr/interrupt.h>

#define MOT_PHUT 60
#define MAX_FREQ 30
#define MAX_ANGLE 180
Servo sv;
LiquidCrystal_I2C lcd(0x3F,16,2);

//Define pin use
//const int varRes = A0;
const int svPin = 9;
const int BT_UP = 2;
const int BT_DOWN = 3;
const int BT_MODE = 4;

//int value_Res = 0;
//int servo_Pos = 0;

//Thời điểm bấm nút cuối cùng ngay trước đó
unsigned long pre_press = 0; 

//Đếm số lần đã bấm nút
static unsigned int number_press = 0;

//Góc quay servo [0,180]
static volatile unsigned int angle;
//Số lần quay servo trong 1 phút [0, 60]
static volatile unsigned int freq = 0;  
//mode = 1: Chỉnh góc, mode = 0: Chỉnh tần số
static volatile bool w_mode = 1;
const float value = 49910;

//Hàm delay 1 giây
void delay_s(unsigned int s){
  while(s--){
    delay(1000);
  }
}



//Thay đổi mode thông qua nút BT_MODE
bool modeChange(){
  if(digitalRead(BT_MODE) == HIGH){
    delay(150);
    if(digitalRead(BT_MODE) == HIGH){
      w_mode = !w_mode;
    return 1;
    }
  }
  else return 0;
}

//Hàm quay servo
void servoTouch(unsigned int angle){
  sv.write(180);
  delay(400);
  sv.write(angle);
  delay(400);
  sv.write(180);
  delay(200);
  number_press++;
}

//In ra màn hình các thông số đang hoạt động
//Thời gian hoạt động
//Góc quay hiện tại của servo
//Số lần bấm
//Mode hiện tại
void condition(){
  //Time
  unsigned long allSeconds=millis()/1000;
  int runHours= allSeconds/3600;
  int secsRemaining=allSeconds%3600;
  int runMinutes=secsRemaining/60;
  int runSeconds=secsRemaining%60;
  char buf[21];
  char buf2[8];
  sprintf(buf,"W:%02d:%02d:%02d",runHours,runMinutes,runSeconds);
  lcd.setCursor(0,0);
  lcd.print(buf);
  
  //Already touch count
  lcd.setCursor(11, 0);
  lcd.print("T:");
  lcd.print(number_press);

  //Angle
  lcd.setCursor(0,1);
  lcd.print("A:");
  lcd.print(angle);
  lcd.print("'");
  //Serial.println(angle);
  
  //Number of press per minute (frequency)
  sprintf(buf2, "F:%d/m ", freq);
  lcd.setCursor(6,1);
  lcd.print(buf2);
  
  lcd.setCursor(13,1);
  lcd.print("M:");
  lcd.print(w_mode);
}


//Hàm ngắt khi bấm BT_UP
void BT_UP_ISR(){
  if(w_mode ==1 ){
    angle += 1;
    Serial.println("Tăng angle lên 1");
    if(angle>MAX_ANGLE) angle = MAX_ANGLE;
    else return;
  }
  else{
    freq += 1;
    Serial.println("Tăng freq lên 1");
    if(freq > MAX_FREQ) freq = MAX_FREQ;
    else return;
  }
}

//Hàm ngắt khi bấm BT_DOWN
void BT_DOWN_ISR(){
  if(w_mode ==1 ){  
    angle -= 1;
    Serial.println("Giảm angle đi 1");
    if(angle<0) angle = 0;
    else return;
  }
  else{
    freq -= 1;
    Serial.println("Giảm freq lên 1");
    if(freq < 0) freq = 0;
    else return;
  }
}

void BT_UP_polling(){
  if(digitalRead(BT_UP) == HIGH){
    delay(150);
    if(digitalRead(BT_UP) == HIGH){
    freq++;
    if(freq>MAX_FREQ) freq = MAX_FREQ;
    }
  }
}

void BT_DOWN_polling(){
  if(digitalRead(BT_DOWN) == HIGH){
    delay(150);
    if(digitalRead(BT_DOWN) == HIGH){
    freq--;
    if(freq < 0) freq = 0;
    }
  }
}

/*
void timer1_init(){
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;

  TCCR1B |= (1 << CS10)|(1 << CS12);  //1024 prescaler
  TCNT1 = value;
  TIMSK1 |= (1 << TOIE1);       //enable timer overflow interrupt
  interrupts();
}

ISR(TIMER1_OVF_vect)                    // interrupt service routine for overflow
{
  TCNT1 = value;                                // preload timer
  condition();
}
*/

void firstTime(){
  angle = 135;
  freq = 30;
  lcd.setCursor(0, 0);
  lcd.print("Touch/min?");
  char fbuf[4];
  while(1){
    lcd.setCursor(12,0);
  sprintf(fbuf, "%d  ",freq);
    lcd.print(fbuf);
  BT_DOWN_polling();
  BT_UP_polling();
    if(digitalRead(BT_MODE) == HIGH){
    delay(150);
    if(digitalRead(BT_MODE) == HIGH)
    {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Start after:");
      lcd.setCursor(13,0);
      lcd.print("3");
      delay_s(1);
      lcd.setCursor(13,0);
      lcd.print("2");
      delay_s(1);
      lcd.setCursor(13,0);
      lcd.print("1");
      delay_s(1);
      lcd.clear();
      break;
    }
  }
  }
}

void freqTouch(){
  unsigned long interval;
  int set_angle = angle;
  int set_freq = freq;
  unsigned long pre_condition, post_condition;
  if(freq == 0) return;
  else if(freq == 1) interval = MOT_PHUT - 1;
  else {
    interval = (unsigned long)MOT_PHUT*1000/(freq-1) - 1;
    for(int i = freq; i>0; i--){
      pre_press = millis();      
      while(1){  
        if(set_angle != angle){
          Serial.println("Thay đổi góc bấm");
          number_press = 0;
          return;
        }
        if(set_freq != freq){
          Serial.println("Thay đổi tốc độ bấm");
          number_press = 0;
          return;
        }
        pre_condition = millis();
        condition();
        post_condition = millis() - pre_condition;
        Serial.print("Thời gian tính toán:");
        Serial.println(post_condition);
        modeChange();
        if((millis()- pre_press)>=interval){
          servoTouch(angle);
          Serial.print("Số lần bấm :");
          Serial.println(number_press);
          break;
        }
      }
    }
  }
}


void setup(){
  //cli();
  sv.attach(svPin);
  lcd.init();                    
  lcd.backlight();
  Serial.begin(115200);
  pinMode(BT_UP|BT_DOWN|BT_MODE, INPUT);
  attachInterrupt(digitalPinToInterrupt(BT_UP), BT_UP_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(BT_DOWN), BT_DOWN_ISR, RISING);
  //timer1_init();
  firstTime();
  //sei();
}

void loop(){
  freqTouch();
  /*
  value_Res = analogRead(varRes);
  servo_Pos = map(value_Res, 0, 1023, 1, 179);
  sv.write(servo_Pos);
  Serial.println(servo_Pos);
  */

}
