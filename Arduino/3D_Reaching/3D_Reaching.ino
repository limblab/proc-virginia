// 3D Reaching Task
// LEDs
int Lin1 = 2;
int Lin2 = 3;
int Lin3 = 4;
int Lout0 = 5;
int Lout1 = 6;
int Lout2 = 7;
int Lout3 = 8;
int Lout4 = 9;
int Lout5 = 10;
int Lout6 = 11;
int Lout7 = 12;
int LEDin[3] = {Lin1,Lin2,Lin3};
int LEDout[8] = {Lout0,Lout1,Lout2,Lout3,Lout4,Lout5,Lout6,Lout7};

// Proximity Sensors
int Pout = A0;

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
}

void loop() {
if (Lin1==LOW && Lin2==LOW && Lin3==LOW){
digitalWrite(Lout0,HIGH);
digitalWrite(LEDout[1,2,3,4,5,6,7],LOW);}

else if (Lin1==LOW && Lin2==LOW && Lin3==HIGH){
digitalWrite(Lout1,HIGH);
digitalWrite(LEDout[0,2,3,4,5,6,7],LOW);}

else if (Lin1==LOW && Lin2==HIGH && Lin3==LOW){
digitalWrite(Lout2,HIGH);
digitalWrite(LEDout[0,1,3,4,5,6,7],LOW);}

else if (Lin1==LOW && Lin2==HIGH && Lin3==HIGH){
digitalWrite(Lout3,HIGH);
digitalWrite(LEDout[0,1,2,4,5,6,7],LOW);}

else if (Lin1==HIGH && Lin2==LOW && Lin3==LOW){
digitalWrite(Lout4,HIGH);
digitalWrite(LEDout[0,1,2,3,5,6,7],LOW);}

else if (Lin1==HIGH && Lin2==LOW && Lin3==HIGH){
digitalWrite(Lout5,HIGH);
digitalWrite(LEDout[0,1,2,3,4,6,5],LOW);}

else if (Lin1==HIGH && Lin2==HIGH && Lin3==LOW){
digitalWrite(Lout6,HIGH);
digitalWrite(LEDout[0,1,2,3,4,5,7],LOW);}

else if (Lin1==HIGH && Lin2==HIGH && Lin3==HIGH){
digitalWrite(Lout7,HIGH);
digitalWrite(LEDout[0,1,2,3,4,5,6],LOW);}
}

