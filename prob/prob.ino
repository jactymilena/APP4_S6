#include <vector>
#include <atomic>
#include <bitset>

const int MAX_SIZE = 80;
const int BASE_TRAME_SIZE = 7;
const int PIN_OUT = 32; 
const int PIN_IN = 33;
const int RECEIVE_CORE = 0;
const int SEND_CORE = 1;
const int ZERO_MODE = 0;
const int ONE_MODE = 1;
const int HALF_PERIOD = 500;
const int THRESHOLD_PERIOD = 280;
const int TIME_BETWEEN_PERIODS = 5000 ; // en micro

// trame components
const uint8_t PREAMBLE = 0b1010101;
const uint8_t START_END = 0b01111110;


const TickType_t xDelay = TIME_BETWEEN_PERIODS / 1000 * portTICK_PERIOD_MS; // en millis

std::vector<uint8_t> buffer{};

int valIn;
int valOut;
unsigned long duration;

unsigned long timer;
unsigned long lastChangeTime = 0;
bool checkPeriod = false;
volatile bool receivedBit = false;
int messageIndex = 0;

// Message State Machine
enum class State {
  IDLE,
  PREAMBLE,
  START,
  HEADER,
  PAYLOAD,
  CRC,
  END
};

State currentState = State::IDLE;

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
    0x07, // payload
    0x00,
    0x00,
    0x7E // end
    };// 

  return trame;
} 

void sendPulse(int value) { // 0, 1
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

  messageIndex++;
  if(messageIndex % 8 == 0) {
    trameAnalyzer();
  }
}

void sendByte(uint8_t bits) {
  int val;
  for(int i = 7; i >= 0; i--) {
    val = (bits >> i) & 0x01;
    // Serial.printf("val %d ", val)
    sendPulse(val);
    vTaskDelay(xDelay);
  }
}

uint8_t convertBufferToByte() {
  uint8_t val;
  int j = 7;
  for(int i = messageIndex - 8; i < messageIndex; i++) {
    val |= buffer[i] << j;
    j--;
  } 
  
  return val;
}

void trameAnalyzer() {
  Serial.printf("buffer size %d\n", buffer.size());
  convertBufferToByte();
  // switch(currentState){
  //   case State::IDLE:



  //   case PREAMBLE:
  //       if(data == "01010101"){
  //           currentState = State::START;
  //       }
  //       break;
        
  //   case START:
  //       if(data == "01111110" && lastState == PREAMBLE) {
  //           currentState = State::HEADER;
  //       } else {
  //           // Revenir à l'état initial
  //           currentState = State::PREAMBLE;
  //       }
  //       break;
        
  //   case HEADER:
  //       currentState = State::Lenght;
  //       break;
    
  //   case Lenght:
  //       currentState = State::PAYLOAD;
  //       break;
    
  //   case PAYLOAD:
  //       currentState = State::CRC;
  //       break;
    
  //   case CRC:
  //       currentState = State::END;
  //       break;
    
  //   case END:
  //       if(data == "01111110")
  //       // Revenir à l'état initial
  //       currentState = State::PREAMBLE;
  //       payload.clear();
  //       payloadLength = 0;
  //       break;
  // }
}

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
        // Serial.printf("if diff %d\n", periodElapse);
    }

    if(receivedBit) {
      receivedBit = false;
      rxVal = digitalRead(PIN_IN);
      
      // Serial.printf("diff %6d rx %d curr %6d last %6d ", periodElapse, rxVal, currentTime, lastChangeTime);

      if(buffer.size() != 0) {
        if(checkPeriod) {
          if(periodElapse >= HALF_PERIOD - THRESHOLD_PERIOD) { // one period has pass
            addBit(!rxVal);
          }
        } else {
          checkPeriod = true;
          // Serial.println(" ");
        }
      } else  { // first bit TODO : a revoir, marche pas pour le zero et le un en meme temps
          addBit(0);
          if(rxVal == 0)
            addBit(!rxVal);
      }

      lastChangeTime = currentTime;
    }
  }
}

void TaskSend(void *pvParameters) {  
  uint8_t message[7] = {
    PREAMBLE, // preamble
    START_END, // start
    0b10000001, // header - Type + Flags
    // size, // header - payload size
    0b11011101, // payload
    0b00000000,
    0b00000000,
    START_END // end
  };

  for(int i = 0; i < 7; i++) {
    sendByte(message[i]);
  } 

  Serial.printf("buffer START");
  for(int i = 0; i < buffer.size(); i++) {
    Serial.printf(" %d", buffer[i]);
  }
  Serial.println(" END received");


  for (;;) {
    vTaskDelay(xDelay);
  }
}


