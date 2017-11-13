// 3D Reaching Task
// LEDs
int Lin1 = 8;
int Lin2 = 9;
int Lin3 = 10;

int Lout1 = 6;
int Lout6 = 1;
int Lout5 = 2;
int Lout0 = 7;
int Lout3 = 4;
int Lout2 = 5;
int Lout7 = 0;
int Lout4 = 3;

// Proximity Sensors
int Pout = 11;
 
int Pin0 = A1;
int Pin1 = A2;
int Pin2 = 13;
int Pin3 = A4;
int Pin4 = A0;
int Pin5 = A5;
int Pin6 = A3;
int Pin7 = 12;

int vrange = 32;

void setup() {
//TCCR0B = TCCR0B & 0b11111000 | 0x04;
//TCCR0B = _BV(CS00);
//Setting   Divisor   Frequency
//0x01    1     31372.55
//0x02    8     3921.16
//0x03      32      980.39
//0x04    64    490.20   <--DEFAULT
//0x05    128     245.10
//0x06      256     122.55
//0x07    1024      30.64

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
int Lin1r = digitalRead(Lin1);
int Lin2r = digitalRead(Lin2);
int Lin3r = digitalRead(Lin3);

if (Lin1r==LOW && Lin2r==LOW && Lin3r==LOW){
digitalWrite(Lout0,LOW);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1r==LOW && Lin2r==LOW && Lin3r==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,LOW);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1r==LOW && Lin2r==HIGH && Lin3r==LOW){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,LOW);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1r==LOW && Lin2r==HIGH && Lin3r==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,LOW);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1r==HIGH && Lin2r==LOW && Lin3r==LOW){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,LOW);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1r==HIGH && Lin2r==LOW && Lin3r==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,LOW);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

else if (Lin1r==HIGH && Lin2r==HIGH && Lin3r==LOW){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,LOW);
digitalWrite(Lout7,HIGH);
}

else if (Lin1r==HIGH && Lin2r==HIGH && Lin3r==HIGH){
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,LOW);
}
else {
digitalWrite(Lout0,HIGH);
digitalWrite(Lout1,HIGH);
digitalWrite(Lout2,HIGH);
digitalWrite(Lout3,HIGH);
digitalWrite(Lout4,HIGH);
digitalWrite(Lout5,HIGH);
digitalWrite(Lout6,HIGH);
digitalWrite(Lout7,HIGH);
}

if (digitalRead(Pin0)==LOW){
analogWrite(Pout,vrange);
}
else if (digitalRead(Pin1)==LOW){
analogWrite(Pout,vrange*2);
}
else if (digitalRead(Pin2)==LOW){
analogWrite(Pout,vrange*3);
}
else if (digitalRead(Pin3)==LOW){
analogWrite(Pout,vrange*4);
}
else if (digitalRead(Pin4)==LOW){
analogWrite(Pout,vrange*5);
}
else if (digitalRead(Pin5)==LOW){
 analogWrite(Pout,vrange*6);
}
else if (digitalRead(Pin6)==LOW){
 analogWrite(Pout,vrange*7);
}
else if (digitalRead(Pin7)==LOW){
 analogWrite(Pout,255);
}
else{
 analogWrite(Pout,0);
}
}

