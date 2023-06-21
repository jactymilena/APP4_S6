#include <vector>
#include <atomic>
#include <mutex>

const int MAX_SIZE = 80;
const int BASE_TRAME_SIZE = 7;
const int PIN_OUT = 32; 
const int PIN_IN = 33;
const int RECEIVE_CORE = 0;
const int SEND_CORE = 1;
const int ZERO_MODE = 0;
const int ONE_MODE = 1;
const int HALF_PERIOD = 500;
const int THRESHOLD_PERIOD = 200;
const int TIME_BETWEEN_PERIODS = 5000 ; // en micro

const TickType_t xDelay = TIME_BETWEEN_PERIODS / 1000 * portTICK_PERIOD_MS; // en millis

std::vector<uint8_t> buffer{};

int valIn;
int valOut;
unsigned long duration;

unsigned long timer;
unsigned long lastChangeTime;
bool checkPeriod;
volatile bool receivedBit;
std::mutex m;

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
  checkPeriod = false;
  receivedBit = false;
}

void loop() { }

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

void sendPulse(int value) {
  digitalWrite(PIN_OUT, value);
  delayMicroseconds(HALF_PERIOD);

  digitalWrite(PIN_OUT, !value);
  delayMicroseconds(HALF_PERIOD);
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

void addBit(int bit) {
  buffer.push_back(bit); 
  checkPeriod = false;

  Serial.printf("buffer START");
  for(int i = 0; i < buffer.size(); i++) {
    Serial.printf(" %d", buffer[i]);
  }
  Serial.println(" END");
}

// void sendTrame(uint8_t *trame, int trame_size) {
//   for(int i = 0; i < trame_size; i++) {

//   }
// }

/*--------------------------------------------------*/
/*-------------------- Interrups -------------------*/
/*--------------------------------------------------*/

void IRAM_ATTR receivePulse() {
  receivedBit = true;
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskReceive(void *pvParameters) { 
  int periodElapse;
  int rxVal;
  
  for (;;) {
    auto currentTime = micros();
    periodElapse = currentTime - lastChangeTime;

    if(!checkPeriod && (periodElapse >= HALF_PERIOD*2 + TIME_BETWEEN_PERIODS - THRESHOLD_PERIOD) && (periodElapse <= HALF_PERIOD*2 + TIME_BETWEEN_PERIODS + THRESHOLD_PERIOD)) {
        checkPeriod = true;
        Serial.printf("if diff %d\n", periodElapse);
    }

    if(receivedBit) {
      receivedBit = false;
      rxVal = digitalRead(PIN_IN);
      
      Serial.printf("diff %6d rx  %d ", periodElapse, rxVal);

      if(buffer.size() != 0) {
        if(checkPeriod) {
          if(periodElapse >= HALF_PERIOD - THRESHOLD_PERIOD || buffer.size() == 0) { // one period has pass
            addBit(!rxVal);
          }
        } else {
          checkPeriod = true;
          Serial.println(" ");
        }
      } else  { // first bit TODO : a revoir, marche pas pour le zero et le un en meme temps
          if(rxVal == 0)
            addBit(!rxVal);
      }

      lastChangeTime = currentTime;
    }
  }
}

void TaskSend(void *pvParameters) {  
  // const TickType_t xDelay = 20;
  for(int i = 0; i < 5; i++) {
      sendOne();
      vTaskDelay(xDelay);
      // sendZero();
      // vTaskDelay(xDelay);
  } // 1 0 1 0 1 0 1 0 1 0 0 0 0 0 0 
    // 1   1 0 1 0 1 0 1 0 0 0 0 0 0

  for(int i = 0; i < 5; i++) {
      sendZero();
      vTaskDelay(xDelay);

      // delayMicroseconds(20);
  }

  for (;;) {
    // sendOne();
    // vTaskDelay(xDelay);
    // sendZero();
    vTaskDelay(xDelay);
  }
}


