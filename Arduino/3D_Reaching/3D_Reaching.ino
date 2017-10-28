// 3D Reaching Task
// LEDs
int Lin1 = 9;
int Lin2 = 10;
int Lin3 = 11;

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

void setup() {
//pinMode(Lin1,INPUT);
//pinMode(Lin2,INPUT);
//pinMode(Lin3,INPUT);
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
}
