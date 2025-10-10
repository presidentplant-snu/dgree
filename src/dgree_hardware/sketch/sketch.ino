#include <string.h>

uint8_t ENA = 7;
uint8_t IN1 = 6;
uint8_t IN2 = 5;

uint8_t ENB = 2;
uint8_t IN3 = 3;
uint8_t IN4 = 4;  


static const uint8_t START = 0xAA;
static const uint8_t END_B = 0x55;
static const uint8_t TYPE_DATA = 0x01;
static const uint8_t TYPE_END  = 0xFF;

enum ParseState {
  WAIT_START,
  READ_TYPE,
  READ_LEN,
  READ_PAYLOAD,
  READ_CHK,
  WAIT_END
};

ParseState state = WAIT_START;
uint8_t pkt_type = 0;
uint8_t pkt_len = 0;
const uint8_t MAX_PAYLOAD = 16;  // 넉넉하게
uint8_t payload_buf[MAX_PAYLOAD];
uint8_t payload_idx = 0;
uint8_t chk_recv = 0;


void motorWrite(int16_t a, int16_t b);
void stopMotors(){
  motorWrite(0,0);
}
void processPacket(uint8_t type, uint8_t len, const uint8_t* data) {
  // 검증: 체크섬
  uint16_t sum = type + len;
  for (uint8_t i = 0; i < len; ++i) sum += data[i];
  uint8_t chk_calc = sum & 0xFF;
  if (chk_calc != chk_recv) {
    // 체크섬 불일치 → 무시
    return;
  }

  if (type == TYPE_DATA) {
    if (len != 4) return; // int16 두 개
    int16_t num1 = (int16_t)( (int16_t)data[0] | ((int16_t)data[1] << 8) ); // little-endian
    int16_t num2 = (int16_t)( (int16_t)data[2] | ((int16_t)data[3] << 8) );
    motorWrite(num1, num2);
  } else if (type == TYPE_END) {
    stopMotors();
  }
}

void resetParser() {
  state = WAIT_START;
  pkt_type = 0;
  pkt_len = 0;
  payload_idx = 0;
  chk_recv = 0;
}


void setup(){

  Serial.begin(9600);

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


int num1=0, num2=0;

void loop() {
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();

    switch (state) {
      case WAIT_START:
        if (b == START) {
          state = READ_TYPE;
        }
        break;

      case READ_TYPE:
        pkt_type = b;
        state = READ_LEN;
        break;

      case READ_LEN:
        pkt_len = b;
        if (pkt_len > MAX_PAYLOAD) {
          // 말이 안 되는 길이 → 리셋
          resetParser();
        } else if (pkt_len == 0) {
          state = READ_CHK;
        } else {
          payload_idx = 0;
          state = READ_PAYLOAD;
        }
        break;

      case READ_PAYLOAD:
        payload_buf[payload_idx++] = b;
        if (payload_idx >= pkt_len) {
          state = READ_CHK;
        }
        break;

      case READ_CHK:
        chk_recv = b;
        state = WAIT_END;
        break;

      case WAIT_END:
        if (b == END_B) {
          // 완전한 패킷 수신
          processPacket(pkt_type, pkt_len, payload_buf);
        }
        // END가 아니어도 어쨌든 리셋(프레이밍 재동기)
        resetParser();
        break;
    }
  }

  // 필요 시 여기서 주기적 작업 수행
}

