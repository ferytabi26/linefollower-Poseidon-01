//definisi header
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);

//definisi button
#define atas digitalRead(3)==0
#define bawah digitalRead(2)==0
#define ok digitalRead(4)==0
#define cancel digitalRead(5)==0

//data
bool mode = 0;
int state = 1;
byte P=0;
byte I=0;
byte D=0;
byte Speed=0;
char buf[20];

//definisi sensor
#define en_sen_kanan 13
#define en_sen_kiri 12
int sensor[6]={A0,A1,A2,A3,A6,A7};
int midVal_ki[6];int putih_ka[6];int hitam_ka[6];
int midVal_ka[6];int putih_ki[6];int hitam_ki[6];
int senValue[12];

//definisi motor
const int en_ka = 11;
const int en_ki = 10;
const int in1_ka = 8;
const int in2_ka = 9;
const int in1_ki = 7;
const int in2_ki = 6;
int pwm_ki,pwm_ka,minPwm = 0,maxPwm = 255,speedm;
//PID
int sumsen;
float error;
bool dirKi,dirKa;
byte Kp,Ki,Kd;
float p,i,d,previous_i,previous_error;
int PID_value;
byte Ts,I1,I2,I3,D1,D2,D3;


void setup() {
  for(int i=0;i<6;i++){
    pinMode(sensor[i],INPUT);
  }
  pinMode(3, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);

  pinMode(en_ka,OUTPUT);
  pinMode(en_ki,OUTPUT);
  pinMode(in1_ka,OUTPUT);
  pinMode(in2_ka,OUTPUT);
  pinMode(in1_ki,OUTPUT);
  pinMode(in2_ki,OUTPUT);
  
  pinMode(en_sen_kanan,OUTPUT);
  pinMode(en_sen_kiri,OUTPUT);
  Serial.begin(115200);
  
  P = EEPROM.read(13);
  I = EEPROM.read(14);
  D = EEPROM.read(15);  
  Speed = EEPROM.read(16);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(4,0);
  lcd.print("POSEIDON");
  delay(1500);
  lcd.clear();
}

void loop() {
  switch(state){
    case 1 :
      lcd.setCursor(0,0);lcd.print("1. Set Mode");
      if(atas){lcd.clear();state++;delay(500);}
      if(ok){
//      for (int i=1;i<13;i++){
//        Serial.println((EEPROM.read(i))*3);
//      }
      lcd.clear();state=11;delay(500);}
    break;
    case 2 : 
      lcd.setCursor(0,0);lcd.print("2. Calibration");
      if(atas){lcd.clear();state++;delay(500);}
      if(bawah){lcd.clear();state--;delay(500);}
      if(ok){lcd.clear();state=21;delay(500);}
    break;
    case 3 :
      lcd.setCursor(0,0);lcd.print("3. Control");
      if(atas){lcd.clear();state++;delay(500);}
      if(bawah){lcd.clear();state--;delay(500);}
      if(ok){lcd.clear();state=31;delay(500);}
    break;
    case 4 :
      lcd.setCursor(0,0);lcd.print("4. Mission");
      analogWrite(en_ka,0);
      digitalWrite(in1_ka,LOW);
      digitalWrite(in2_ka,LOW);
      analogWrite(en_ki,0);
      digitalWrite(in1_ki,LOW);
      digitalWrite(in2_ki,LOW);
      if(atas){lcd.clear();state++;delay(500);}
      if(bawah){lcd.clear();state--;delay(500);}
    break;
    case 5 :
      lcd.setCursor(0,0);lcd.print("5. Start !!!");
      if(cancel){lcd.clear();state=51;delay(500);}
//      if(atas){lcd.clear();state++;delay(500);}
//      if(bawah){lcd.clear();state--;delay(500);}
      if(ok){
        Kp=EEPROM.read(13);
        Ki=EEPROM.read(14);
        Kd=EEPROM.read(15);
        Ts = EEPROM.read(100);
        speedm = EEPROM.read(16);
        mode = EEPROM.read(0);
        if(mode==0){
          for (int i=1;i<7;i++){
            midVal_ka[i-1]=((EEPROM.read(i))*3);
          }
          for (int i=7;i<13;i++){
            midVal_ki[i-7]=((EEPROM.read(i))*3);
          }
          bool keadaan = 1;
        while(keadaan){
          digitalWrite(en_sen_kiri,LOW);
          digitalWrite(en_sen_kanan,HIGH);
          for(int i=0;i<6;i++){
            if(analogRead(sensor[i])>midVal_ka[i]) senValue[i]=1; else senValue[i]=0;
          }
          if(cancel){
            keadaan = 0;
            state = 4;}
          digitalWrite(en_sen_kiri,HIGH);
          digitalWrite(en_sen_kanan,LOW);
          for(int i=5,j=6;i>=0,j<12;i--,j++){
            if(analogRead(sensor[i])>midVal_ki[j-6]) senValue[j]=1; else senValue[j]=0;
          }
          int sumSen = 0;
           sumSen=(senValue[0]*1)+(senValue[1]*2)+(senValue[2]*4)+(senValue[3]*8)+(senValue[4]*16)+(senValue[5]*32)+
           (senValue[6]*64)+(senValue[7]*128)+(senValue[8]*248)+(senValue[9]*512)+(senValue[10]*1024)+(senValue[11]*2048);
          switch(sumSen){ 
          case 0b000001111111 : error=20;dirKi=1;dirKa=1; break; 
          case 0b000000111111 : error=20;dirKi=1;dirKa=1; break; 
          case 0b000000011111 : error=20;dirKi=1;dirKa=1; break; 
          case 0b000000001111 : error=20;dirKi=1;dirKa=1; break; 
          case 0b000000000001 : error=11;dirKi=1;dirKa=1; break; 
          case 0b000000000111 : error=10;dirKi=1;dirKa=1; break; 
          case 0b000000000011 : error=10;dirKi=1;dirKa=1; break; 
          case 0b000000000010 : error=9;dirKi=1;dirKa=1; break; 
          case 0b000000001110 : error=8;dirKi=1;dirKa=1; break; 
          case 0b000000000110 : error=8;dirKi=1;dirKa=1; break; 
          case 0b000000000100 : error=7;dirKi=1;dirKa=1; break; 
          case 0b000000011100 : error=6;dirKi=1;dirKa=1; break; 
          case 0b000000001100 : error=6;dirKi=1;dirKa=1; break; 
          case 0b000000001000 : error=5;dirKi=1;dirKa=1; break; 
          case 0b000000111000 : error=4;dirKi=1;dirKa=1; break; 
          case 0b000000011000 : error=4;dirKi=1;dirKa=1; break; 
          case 0b000000010000 : error=3;dirKi=1;dirKa=1; break; 
          case 0b000001110000 : error=2;dirKi=1;dirKa=1; break; 
          case 0b000000110000 : error=2;dirKi=1;dirKa=1; break; 
          case 0b000000100000 : error=1;dirKi=1;dirKa=1; break; 
          case 0b000001100000 : error=0;dirKi=1;dirKa=1; break; 
          case 0b000001000000 : error=-1;dirKi=1;dirKa=1; break; 
          case 0b000011000000 : error=-2;dirKi=1;dirKa=1; break; 
          case 0b000011100000 : error=-2;dirKi=1;dirKa=1; break; 
          case 0b000010000000 : error=-3;dirKi=1;dirKa=1; break; 
          case 0b000110000000 : error=-4;dirKi=1;dirKa=1; break; 
          case 0b000111000000 : error=-4;dirKi=1;dirKa=1; break; 
          case 0b000100000000 : error=-5;dirKi=1;dirKa=1; break; 
          case 0b001100000000 : error=-6;dirKi=1;dirKa=1; break; 
          case 0b001110000000 : error=-6;dirKi=1;dirKa=1; break; 
          case 0b001000000000 : error=-7;dirKi=1;dirKa=1; break; 
          case 0b011000000000 : error=-8;dirKi=1;dirKa=1; break; 
          case 0b011100000000 : error=-8;dirKi=1;dirKa=1; break; 
          case 0b010000000000 : error=-9;dirKi=1;dirKa=1; break; 
          case 0b110000000000 : error=-10;dirKi=1;dirKa=1; break; 
          case 0b111000000000 : error=-10;dirKi=1;dirKa=1; break; 
          case 0b100000000000 : error=-11;dirKi=1;dirKa=1; break; 
          case 0b111100000000 : error=-20;dirKi=1;dirKa=1; break; 
          case 0b111110000000 : error=-20;dirKi=1;dirKa=1; break; 
          case 0b111111000000 : error=-20;dirKi=1;dirKa=1; break; 
          case 0b111111100000 : error=-20;dirKi=1;dirKa=1; break; 
          case 0b000000000000 : if(previous_error<0){dirKi=0;dirKa=1;}
                                else if(previous_error>0){dirKi=1;dirKa=0;}break;
                                 
          default : 
          error = previous_error;
          break;
          }
//          lcd.clear();lcd.print(error);
//          lcd.setCursor(0,1);lcd.print(pwm_ki);
//          lcd.setCursor(8,1);lcd.print(pwm_ka);
//          delay(10);
          p = error;
          i = i+previous_i;
          d = error - previous_error;
          PID_value = (Kp*p)+(Ki*i)+(Kd*d);
          p = Kp * error;
          D1 = Kd * 10;
          D2 = D1 / Ts;
          D3 = error-previous_error;
          d = D2 * D3;
          I1 = Ki/10;
          I2 = error + previous_error;
          I3  = I1*I2;
          i = I3 * Ts;
          PID_value = p + i + d;
          previous_i = i;
          previous_error = error;
          pwm_ka = (speedm-15) - PID_value;
          pwm_ki = speedm + PID_value;
          if(pwm_ka < 0){pwm_ka=0;}
          if(pwm_ka > 255){pwm_ka=255;} 
          if(pwm_ki < 0){pwm_ki=0;}
          if(pwm_ki > 255){pwm_ki=255;}
          
          motorku(pwm_ki,pwm_ka,dirKi,dirKa,en_ka,en_ki);
        }
        }
    }
    break;
    case 6 :
      lcd.setCursor(0,0);lcd.print("6. Reset");
      if(atas){lcd.clear();state++;delay(500);}
      if(bawah){lcd.clear();state--;delay(500);}
    break;
    //submenu setmode
    case 11 :
      lcd.setCursor(0,0);lcd.print("Mode = ");lcd.print(mode);
      lcd.setCursor(0,1);lcd.print("0=Normal 1=Misi");
      if(atas){mode=1;}
      if(bawah){mode=0;}
      if(ok){EEPROM.put(0,mode);state=1;lcd.clear();lcd.setCursor(6,0);lcd.print("save");delay(1000);lcd.clear();}
      if(cancel){lcd.clear();lcd.print("MODE:");lcd.print(EEPROM.read(0));delay(1000);state=1;}
    break; 
    //submenu calibration
    case 21 :
      lcd.setCursor(0,0);lcd.print("Kanan<<");
      lcd.setCursor(0,1);lcd.print("Kiri");
      lcd.setCursor(8,0);lcd.print("Saving");
      if(atas){lcd.clear();state=21;delay(500);}
      if(bawah){lcd.clear();state=22;delay(500);}
      if(cancel){lcd.clear();state=2;delay(500);}
      if(ok){lcd.clear();state=211;delay(500);}
    break; 
    case 22 :
      lcd.setCursor(0,0);lcd.print("Kanan");
      lcd.setCursor(0,1);lcd.print("Kiri <<");
      lcd.setCursor(8,0);lcd.print("Saving");
      if(atas){lcd.clear();state=21;delay(500);}
      if(bawah){lcd.clear();state=23;delay(500);}
      if(cancel){lcd.clear();state=2;delay(500);}
      if(ok){lcd.clear();state=221;delay(500);}
    break; 
    case 23 :
      lcd.setCursor(0,0);lcd.print("Kanan");
      lcd.setCursor(0,1);lcd.print("Kiri");
      lcd.setCursor(8,0);lcd.print("Saving<<");
      if(atas){lcd.clear();state=22;delay(500);}
      if(bawah){lcd.clear();state=23;delay(500);}
      if(cancel){lcd.clear();state=2;delay(500);}
      if(ok){lcd.clear();state=231;delay(500);}
    break; 
    //submenu baca kanan
    case 211:
      lcd.setCursor(0,0);lcd.print("Baca Hitam <<");
      lcd.setCursor(0,1);lcd.print("Baca Putih ");
      if(atas){lcd.clear();state=211;delay(500);}
      if(bawah){lcd.clear();state=212;delay(500);}
      if(cancel){lcd.clear();state=21;delay(500);}
      if(ok){
        for(int i=0;i<100;i++){
          digitalWrite(en_sen_kiri,LOW);
          digitalWrite(en_sen_kanan,HIGH);
          for(int i=0;i<6;i++){
            hitam_ka[i]=analogRead(sensor[i]);
            delayMicroseconds(100);
          }
          delayMicroseconds(100);
        }
        digitalWrite(en_sen_kiri,LOW);
        digitalWrite(en_sen_kanan,LOW);
        for(int i=0;i<6;i++){
          Serial.print(hitam_ka[i]);Serial.print("  ");
        }
        Serial.println();
        lcd.clear();lcd.setCursor(0,0);lcd.print("tersimpan");delay(500);
      }
    break;
    case 212:
      lcd.setCursor(0,0);lcd.print("Baca Hitam ");
      lcd.setCursor(0,1);lcd.print("Baca Putih <<");
      if(atas){lcd.clear();state=211;delay(500);}
      if(bawah){lcd.clear();state=212;delay(500);}
      if(cancel){lcd.clear();state=21;delay(500);}
      if(ok){
        for(int i=0;i<100;i++){
          digitalWrite(en_sen_kiri,LOW);
          digitalWrite(en_sen_kanan,HIGH);
          for(int i=0;i<6;i++){
            putih_ka[i]=analogRead(sensor[i]);
            delayMicroseconds(100);
          }
          delayMicroseconds(100);
        }
        digitalWrite(en_sen_kiri,LOW);
        digitalWrite(en_sen_kanan,LOW);
        for(int i=0;i<6;i++){
          Serial.print(putih_ka[i]);Serial.print("  ");
        }
        Serial.println();
        lcd.clear();lcd.setCursor(0,0);lcd.print("tersimpan");delay(500);
      }
    break;
    //submenu baca kiri
    case 221:
      lcd.setCursor(0,0);lcd.print("Baca Hitam <<");
      lcd.setCursor(0,1);lcd.print("Baca Putih ");
      if(atas){lcd.clear();state=221;delay(500);}
      if(bawah){lcd.clear();state=222;delay(500);}
      if(cancel){lcd.clear();state=22;delay(500);}
      if(ok){
        for(int i=0;i<100;i++){
          digitalWrite(en_sen_kiri,HIGH);
          digitalWrite(en_sen_kanan,LOW);
          for(int i=0;i<6;i++){
            hitam_ki[i]=analogRead(sensor[i]);
            delayMicroseconds(100);
          }
          delayMicroseconds(100);
        }
        digitalWrite(en_sen_kiri,LOW);
        digitalWrite(en_sen_kanan,LOW);
        for(int i=0;i<6;i++){
          Serial.print(hitam_ki[i]);Serial.print("  ");
        }
        Serial.println();
        lcd.clear();lcd.setCursor(0,0);lcd.print("tersimpan");delay(500);
      }
    break;
    case 222:
      lcd.setCursor(0,0);lcd.print("Baca Hitam ");
      lcd.setCursor(0,1);lcd.print("Baca Putih <<");
      if(atas){lcd.clear();state=221;delay(500);}
      if(bawah){lcd.clear();state=222;delay(500);}
      if(cancel){lcd.clear();state=22;delay(500);}
      if(ok){
        for(int i=0;i<100;i++){
          digitalWrite(en_sen_kiri,HIGH);
          digitalWrite(en_sen_kanan,LOW);
          for(int i=0;i<6;i++){
            putih_ki[i]=analogRead(sensor[i]);
            delayMicroseconds(100);
          }
          delayMicroseconds(100);
        }
        digitalWrite(en_sen_kiri,LOW);
        digitalWrite(en_sen_kanan,LOW);
        for(int i=0;i<6;i++){
          Serial.print(putih_ki[i]);Serial.print("  ");
        }
        Serial.println();
        lcd.clear();lcd.setCursor(0,0);lcd.print("tersimpan");delay(500);
      }
    break;
    case 231 :
      for(int i=0;i<6;i++){
        midVal_ka[i]=(putih_ka[i]+hitam_ka[i])/2;
        EEPROM.update(i+1,(midVal_ka[i]/3));
        Serial.println(midVal_ka[i]);
      }
      for(int i=0;i<6;i++){
        midVal_ki[i]=(putih_ki[i]+hitam_ki[i])/2;
        EEPROM.update(i+7,(midVal_ki[i])/3);
        Serial.println(midVal_ki[i]);
      }
//      for (int i=1;i<7;i++){
//        midVal_ka[i-1]=((EEPROM.read(i))*3);
//      }
//      for (int i=7;i<13;i++){
//        midVal_ki[i-7]=((EEPROM.read(i))*3);
//      }
//      
//    while(true){
//    digitalWrite(en_sen_kiri,LOW);
//    digitalWrite(en_sen_kanan,HIGH);
//    for(int i=0;i<6;i++){
//      if(analogRead(sensor[i])>midVal_ka[i]) senValue[i]=1; else senValue[i]=0;
//    }
//    digitalWrite(en_sen_kiri,HIGH);
//    digitalWrite(en_sen_kanan,LOW);
//    for(int i=5,j=6;i>=0,j<12;i--,j++){
//      if(analogRead(sensor[i])>midVal_ki[j-6]) senValue[j]=1; else senValue[j]=0;
//    }
//    for(int i=0;i<12;i++){
//      Serial.print(senValue[i]);
//    }
//    Serial.println();
//    }
    lcd.clear();lcd.setCursor(4,0);lcd.print("tersimpan");delay(1000);lcd.clear();state=23;
    break;
    //submenu control
    case 31 :
      lcd.setCursor(0,0);lcd.print("a. PID   <<");
      lcd.setCursor(0,1);lcd.print("b. Speed");
      if(atas){lcd.clear();state=31;delay(500);}
      if(bawah){lcd.clear();state=32;delay(500);}
      if(cancel){lcd.clear();state=3;delay(500);}
      if(ok){lcd.clear();state=311;delay(500);}
    break; 
    case 32 :
      lcd.setCursor(0,0);lcd.print("a. PID");
      lcd.setCursor(0,1);lcd.print("b. Speed <<");
      if(atas){lcd.clear();state=31;delay(500);}
      if(bawah){lcd.clear();state=32;delay(500);}
      if(cancel){lcd.clear();state=3;delay(500);}
      if(ok){lcd.clear();state=321;delay(500);}
    break; 
    //submenu control PID
    case 311 :
      lcd.setCursor(0,0);lcd.print("Kp=");lcd.print(P);lcd.print("<<");
      lcd.setCursor(0,1);lcd.print("Ki=");lcd.print(I);
      lcd.setCursor(8,0);lcd.print("Kd=");lcd.print(D);
      lcd.setCursor(8,1);lcd.print("Simpan");
      if(atas){lcd.clear();state=311;delay(500);}
      if(bawah){lcd.clear();state=312;delay(500);}
      if(ok){lcd.clear();P++;delay(300);}
      if(cancel){lcd.clear();P--;delay(300);}
    break;
    case 312 :
      lcd.setCursor(0,0);lcd.print("Kp=");lcd.print(P);
      lcd.setCursor(0,1);lcd.print("Ki=");lcd.print(I);lcd.print("<<");
      lcd.setCursor(8,0);lcd.print("Kd=");lcd.print(D);
      lcd.setCursor(8,1);lcd.print("Simpan");
      if(atas){lcd.clear();state=311;delay(500);}
      if(bawah){lcd.clear();state=313;delay(500);}
      if(ok){lcd.clear();I++;delay(300);}
      if(cancel){lcd.clear();I--;delay(300);}
    break;
    case 313 :
      lcd.setCursor(0,0);lcd.print("Kp=");lcd.print(P);
      lcd.setCursor(0,1);lcd.print("Ki=");lcd.print(I);
      lcd.setCursor(8,0);lcd.print("Kd=");lcd.print(D);lcd.print("<<");
      lcd.setCursor(8,1);lcd.print("Simpan");
      if(atas){lcd.clear();state=312;delay(500);}
      if(bawah){lcd.clear();state=314;delay(500);}
      if(ok){lcd.clear();D++;delay(300);}
      if(cancel){lcd.clear();D--;delay(300);}
    break;
    case 314 :
      lcd.setCursor(0,0);lcd.print("Kp=");lcd.print(P);
      lcd.setCursor(0,1);lcd.print("Ki=");lcd.print(I);
      lcd.setCursor(8,0);lcd.print("Kd=");lcd.print(D);
      lcd.setCursor(8,1);lcd.print("Simpan");lcd.print("<<");
      if(atas){lcd.clear();state=313;delay(500);}
      if(bawah){lcd.clear();state=314;delay(500);}
      if(ok){
        EEPROM.update(13,P);
        EEPROM.update(14,I);
        EEPROM.update(15,D);
        lcd.clear();lcd.print("Tersimpan");delay(1000);lcd.clear();state=31;}
      if(cancel){
        lcd.clear();
        lcd.setCursor(0,0);lcd.print("Kp=");lcd.print(EEPROM.read(13));
        lcd.setCursor(0,1);lcd.print("Ki=");lcd.print(EEPROM.read(14));
        lcd.setCursor(8,0);lcd.print("Kd=");lcd.print(EEPROM.read(15));delay(1000);lcd.clear();
        state=31;}
    break;
    //submenu control speed
    case 321 :
      lcd.setCursor(0,0);lcd.print("Ts=");lcd.print(Ts);lcd.print("<<");
      lcd.setCursor(0,1);lcd.print("Speed=");lcd.print(Speed);
      lcd.setCursor(8,0);lcd.print("simpan");
      if(atas){lcd.clear();state=321;delay(500);}
      if(bawah){lcd.clear();state=322;delay(500);}
      if(ok){lcd.clear();Ts++;delay(300);}
      if(cancel){lcd.clear();Ts--;delay(300);}
      break;
    case 322 :
      lcd.setCursor(0,0);lcd.print("Ts=");lcd.print(Ts);
      lcd.setCursor(0,1);lcd.print("Speed=");lcd.print(Speed);;lcd.print("<<");
      lcd.setCursor(8,0);lcd.print("simpan");
      if(atas){lcd.clear();state=321;delay(500);}
      if(bawah){lcd.clear();state=323;delay(500);}
      if(ok){lcd.clear();Speed++;delay(300);}
      if(cancel){lcd.clear();Speed--;delay(300);}
    break;
    case 323 :
      lcd.setCursor(0,0);lcd.print("Ts=");lcd.print(Ts);
      lcd.setCursor(0,1);lcd.print("Speed=");lcd.print(Speed);
      lcd.setCursor(8,0);lcd.print("simpan");lcd.print("<<");
      if(atas){lcd.clear();state=322;delay(500);}
      if(bawah){lcd.clear();state=323;delay(500);}
      if(ok){lcd.clear();
        EEPROM.update(100,Ts);
        EEPROM.update(16,Speed);
        lcd.clear();lcd.setCursor(0,0);lcd.print("Tersimpan");delay(1000);lcd.clear();;state=32;}
    break;
    case 51 :
      for (int i=1;i<7;i++){
            midVal_ka[i-1]=((EEPROM.read(i))*3);
          }
          for (int i=7;i<13;i++){
            midVal_ki[i-7]=((EEPROM.read(i))*3);
          }
          digitalWrite(en_sen_kiri,LOW);
          digitalWrite(en_sen_kanan,HIGH);
          for(int i=0;i<6;i++){
            if(analogRead(sensor[i])>midVal_ka[i]) senValue[i]=1; else senValue[i]=0;
          }
          digitalWrite(en_sen_kiri,HIGH);
          digitalWrite(en_sen_kanan,LOW);
          for(int i=5,j=6;i>=0,j<12;i--,j++){
            if(analogRead(sensor[i])>midVal_ki[j-6]) senValue[j]=1; else senValue[j]=0;
          }
      sprintf(buf,"%1i%1i%1i%1i%1i%1i%1i%1i%1i%1i%1i%1i ",senValue[11],senValue[10],senValue[9],senValue[8],senValue[7],senValue[6],
      senValue[5],senValue[4],senValue[3],senValue[2],senValue[1],senValue[0]);
      lcd.setCursor(0,0);
      lcd.print(buf);
      if(cancel){lcd.clear();state=5;delay(300);}
      break;   
    default:
      state = 1;
    break;
    
      }

}
void motorku(int pwm_ki,int pwm_ka,bool dirki,bool dirka,int en_ka,int en_ki){
  if(dirki==1&&dirka==1){
    analogWrite(en_ka,pwm_ka);
    digitalWrite(in1_ka,HIGH);
    digitalWrite(in2_ka,LOW);
    analogWrite(en_ki,pwm_ki);
    digitalWrite(in1_ki,HIGH);
    digitalWrite(in2_ki,LOW); 
  }
  else if(dirki==1&&dirka==0){
    analogWrite(en_ka,255);
    digitalWrite(in1_ka,LOW);
    digitalWrite(in2_ka,HIGH);
    analogWrite(en_ki,255);
    digitalWrite(in1_ki,HIGH);
    digitalWrite(in2_ki,LOW); 
  }
  else if(dirki==0&&dirka==1){
    analogWrite(en_ka,255);
    digitalWrite(in1_ka,HIGH);
    digitalWrite(in2_ka,LOW);
    analogWrite(en_ki,255);
    digitalWrite(in1_ki,LOW);
    digitalWrite(in2_ki,HIGH); 
  }
  else if(dirki==0&&dirka==0){
    analogWrite(en_ka,pwm_ka);
    digitalWrite(in1_ka,LOW);
    digitalWrite(in2_ka,LOW);
    analogWrite(en_ki,pwm_ki);
    digitalWrite(in1_ki,LOW);
    digitalWrite(in2_ki,LOW); 
  }
  
}
