#include <vector>

// vector<uint8_t> 

const int MAX_SIZE = 80;
const int BASE_TRAME_SIZE = 7;
const int PIN_OUT = 33; 
const int PIN_IN = 32;
const int RISING = 0;
const int FALLING = 1;


void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);

  // attachInterrupt(digitalPinToInterrupt(PIN_IN), blink, RISING);
  
}

// void receiveRising() {

// }

uint8_t* createTrame(uint8_t* data, int size) {
  // std::vector<uint8_t> trame( (BASE_TRAME_SIZE + size) % MAX_SIZE);
  int trame_size = BASE_TRAME_SIZE + size;
  if (trame_size > MAX_SIZE) trame_size = MAX_SIZE;

  uint8_t trame[trame_size] = {
    0x55, // preamble
    0x7E, // start
    0x00, // header - Type + Flags
    (uint8_t) size, // header - payload size
    0x7E // end
  };

  return trame;
}

void sendPulse(int mode) { // falling (1), rising (0)
  digitalWrite(PIN_OUT, mode);
  delayMicroseconds(100);
  digitalWrite(PIN_OUT, !mode);
}

void loop() {
  // put your main code here, to run repeatedly:

}


