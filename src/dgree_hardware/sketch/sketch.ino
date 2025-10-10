uint8_t ENA = 7;
uint8_t IN1 = 6;
uint8_t IN2 = 5;

uint8_t ENB = 2;
uint8_t IN3 = 3;
uint8_t IN4 = 4;

void motorWrite(int16_t a, int16_t b);

void setup(){
  for(uint8_t i =2; i<8; i++){
    pinMode(i, OUTPUT);
  }

  TCCR3B &= 0b11111000;
  TCCR4B &= 0b11111000;
  TCCR3B |= 0b00000101;
  TCCR4B |= 0b00000101;
}

void motorWrite(int16_t a, int16_t b){
  if(a>0){
    digitalWrite(IN1,HIGH);
    digitalWrite(IN2,LOW);
  }
  else{
    digitalWrite(IN1,LOW);
    digitalWrite(IN2,HIGH);
  }

  if(b>0){
    digitalWrite(IN3,HIGH);
    digitalWrite(IN4,LOW);
  }
  else{
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,HIGH);
  }

  analogWrite(ENA,abs(a));
  analogWrite(ENB,abs(b));
}


void loop(){
  motorWrite(120,-100);
}
