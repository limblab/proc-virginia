
int Pout = A0;

void setup() {
pinMode(Pout,INPUT);
Serial.begin(9600);

}

void loop() {
int val = digitalRead(Pout);
Serial.println(val);
}
