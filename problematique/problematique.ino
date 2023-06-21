#include <vector>
#include <mutex>
#include <atomic>

const int MAX_SIZE = 80;
const int BASE_TRAME_SIZE = 7;
const int PIN_OUT = 32; 
const int PIN_IN = 33;
const int RECEIVE_CORE = 0;
const int SEND_CORE = 1;
const int ZERO_MODE = 0;
const int ONE_MODE = 1;
const int HALF_PERIOD = 500;
const int THRESHOLD_PERIOD = 40;

std::vector<uint8_t> buffer{};

int valIn;
int valOut;
unsigned long duration;

unsigned long timer;
unsigned long lastChangeTime;
bool checkPeriod;
std::atomic<bool> receivedBit;

std::mutex receivedBitMutex;

// Define functions
void receivePulse();
void TaskReceive(void *pvParameters);
void TaskSend(void *pvParameters);

void setup() {
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);

  Serial.begin(115200);
  Serial.println("START ------------------------");

  // Receive interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_IN), receivePulse, CHANGE);

  // Receive Task
  xTaskCreate(TaskReceive, "Receive Trame", 2048, NULL, 2,  NULL);
  // Send Task
  xTaskCreate(TaskSend, "Send Trame", 2048, NULL, 3,  NULL);

  lastChangeTime = 0;
  checkPeriod = true;
  receivedBit = false;
}

void loop() {
  // Serial.println("alllooooo");
  // put your main code here, to run repeatedly:
  // unsigned long duration = pulseIn(PIN_IN, HIGH);
  // Serial.printf("pulse duration %d ", duration);

  // timer = micros();
  // valIn = digitalRead(PIN_IN); 
  // valOut = digitalRead(PIN_OUT); 

  // Serial.printf("in %d ", valIn);
  // Serial.printf("buffer size %d\n", buffer.size());

  // if(buffer.size() > 10) {
  //   Serial.println(" START buffer print");
  //   for(int i = 0; i < buffer.size(); i++) {
  //     Serial.printf(" %d", buffer[i]);
  //   }
  //   Serial.println(" END buffer print");
  // }
}

uint8_t* createTrame(uint8_t* data, uint8_t size) {
  // std::vector<uint8_t> trame( (BASE_TRAME_SIZE + size) % MAX_SIZE);
  int trame_size = BASE_TRAME_SIZE + size;
  if (trame_size > MAX_SIZE) trame_size = MAX_SIZE;

  uint8_t trame[trame_size] = {
    0x55, // preamble
    0x7E, // start
    0x00, // header - Type + Flags
    size, // header - payload size
    0x7E // end
  };// 

  return trame;
} 

void sendZero() {
  digitalWrite(PIN_OUT, LOW);
  delayMicroseconds(HALF_PERIOD);

  digitalWrite(PIN_OUT, HIGH);
  delayMicroseconds(HALF_PERIOD);
}

void sendOne() {
  digitalWrite(PIN_OUT, HIGH);
  delayMicroseconds(HALF_PERIOD);

  digitalWrite(PIN_OUT, LOW);
  delayMicroseconds(HALF_PERIOD);
}

// void sendTrame(uint8_t *trame, int trame_size) {
//   for(int i = 0; i < trame_size; i++) {

//   }
// }

/*--------------------------------------------------*/
/*-------------------- Interrups -------------------*/
/*--------------------------------------------------*/

void IRAM_ATTR receivePulse() { 
  int rxVal = digitalRead(PIN_IN);
  auto currentTime = micros();
  if(checkPeriod) {
    int diff = currentTime - lastChangeTime;

    if((diff >= HALF_PERIOD - THRESHOLD_PERIOD) && (diff <= HALF_PERIOD + THRESHOLD_PERIOD)) { // one period has pass
      checkPeriod = false;
      // read pin et mettre dans buffer selon rising ou falling sachant que on est au 2eme change
      buffer.push_back(!rxVal); 
      receivedBit = true;
    }
    // si plus qu'une periode
    //    push selon rising et falling
    //    checkPeriod = true
  } else {
    checkPeriod = true;
  }

  lastChangeTime = currentTime;
  // Add bit to vector
  // Notify task
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskReceive(void *pvParameters) { 
  while(true) {
    if(receivedBit) {
      receivedBit = false;
      Serial.printf("buffer START");
      for(int i = 0; i < buffer.size(); i++) {
        Serial.printf(" %d", buffer[i]);
      }
      Serial.println(" END");
    }
  }
}

void TaskSend(void *pvParameters) {  
  const TickType_t xDelay = 500;
  for(int i = 0; i < 5; i++) {
      sendOne();
      vTaskDelay(xDelay);
  }

  while(true) {
    // sendOne();
    vTaskDelay(xDelay);
  }
}


