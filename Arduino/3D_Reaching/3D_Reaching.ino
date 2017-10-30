// 3D Reaching Task
// LEDs
int Lin1 = 8;
int Lin2 = 9;
int Lin3 = 12;

int Lout1 = 6;
int Lout6 = 1;
int Lout5 = 2;
int Lout0 = 7;
int Lout3 = 4;
int Lout2 = 5;
int Lout7 = 0;
int Lout4 = 3;

// Proximity Sensors
int Pout = A0;
int Pin0 = A1;
int Pin1 = A2;
int Pin2 = A3;
int Pin3 = A4;
int Pin4 = A5;
int Pin5 = 10;
int Pin6 = 11;
int Pin7 = 13;
int vrange = 32;

void setup() {
pinMode(Lin1,INPUT);
pinMode(Lin2,INPUT);
pinMode(Lin3,INPUT);
pinMode(Lout0,OUTPUT);
pinMode(Lout1,OUTPUT);
pinMode(Lout2,OUTPUT);
pinMode(Lout3,OUTPUT);
pinMode(Lout4,OUTPUT);
pinMode(Lout5,OUTPUT);
pinMode(Lout6,OUTPUT);
pinMode(Lout7,OUTPUT);
//digitalWrite(Lin1,LOW);
//digitalWrite(Lin2,LOW);
//digitalWrite(Lin3,LOW);

pinMode(Pout,OUTPUT);
pinMode(Pin0,INPUT);
pinMode(Pin1,INPUT);
pinMode(Pin2,INPUT);
pinMode(Pin3,INPUT);
pinMode(Pin4,INPUT);
pinMode(Pin5,INPUT);
pinMode(Pin6,INPUT);
pinMode(Pin7,INPUT);

//Serial.begin(9600);
}

void loop() {

if (Lin1==LOW && Lin2==LOW && Lin3==LOW){
digitalWrite(Lout0,LOW);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1==LOW && Lin2==LOW && Lin3==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,LOW);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1==LOW && Lin2==HIGH && Lin3==LOW){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,LOW);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1==LOW && Lin2==HIGH && Lin3==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,LOW);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1==HIGH && Lin2==LOW && Lin3==LOW){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,LOW);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1==HIGH && Lin2==LOW && Lin3==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,LOW);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1==HIGH && Lin2==HIGH && Lin3==LOW){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,LOW);
digitalWrite(Lout7,HIGH);
}

else if (Lin1==HIGH && Lin2==HIGH && Lin3==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,LOW);
}

if (analogRead(Pin0)>=512){
analogWrite(Pout,0);
}
if (analogRead(Pin1)>=512){
analogWrite(Pout,vrange);
}
if (analogRead(Pin2)>=512){
analogWrite(Pout,vrange*2);
}
if (analogRead(Pin3)>=512){
analogWrite(Pout,vrange*3);
}
if (analogRead(Pin4)>=512){
analogWrite(Pout,vrange*4);
}
if (analogRead(Pin5)>=512){
analogWrite(Pout,vrange*5);
}
if (analogRead(Pin6)>=512){
analogWrite(Pout,vrange*6);
}
if (analogRead(Pin7)>=512){
analogWrite(Pout,255);
}

}
