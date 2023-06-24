#include <vector>
#include <atomic>
#include <bitset>
#include "CRC16.h"

// Trame sizes
const int MAX_SIZE = 80;
const int BASE_TRAME_SIZE = 7;
// RX and TX pins
const int PIN_OUT = 32; 
const int PIN_IN = 33;
// 
const int HALF_PERIOD = 500;            // en micro
const int THRESHOLD_PERIOD = 280;       // en micro
const int TIME_BETWEEN_PERIODS = 5000 ; // en micro
const TickType_t xDelay = TIME_BETWEEN_PERIODS / 1000 * portTICK_PERIOD_MS; // en millis


// trame components
const uint8_t PREAMBLE = 0b1010101;
const uint8_t START_END = 0b01111110;

std::vector<uint8_t> buffer{};
std::vector<uint8_t> payload{}; // on a deja un index, pourrait etre une array

unsigned long timer;
unsigned long lastChangeTime = 0;
bool checkPeriod = false;
volatile bool receivedBit = false;
int messageIndex = 0;
int currentPayloadSize;
int payloadSize;

// Message State Machine
enum class State {
  IDLE,
  PREAMBLE,
  START,
  HEADER_FLAG,
  HEADER_SIZE,
  PAYLOAD,
  CRC,
  END
  // FINAL_STATE
};

State currentState = State::IDLE;
State lastState = State::IDLE;

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
  xTaskCreate(TaskReceive, "Receive Trame", 2048, NULL, 3,  NULL);
  // Send Task
  xTaskCreate(TaskSend, "Send Trame", 2048, NULL, 2,  NULL);
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

void printBuffer() {
  Serial.printf("buffer START");
  for(int i = 0; i < buffer.size(); i++) {
    Serial.printf(" %d", buffer[i]);
  }
  Serial.println(" END received");
}

void initState() {
  currentState = State::IDLE;
  currentPayloadSize = 0;
  payloadSize = 0;
  buffer.clear();
}

void trameAnalyzer() {
  uint8_t data = convertBufferToByte();
  CRC16 crc;
  static uint16_t crcVal;
  
  // Serial.printf("currState %d data %d\n", currentState, data);
  // TODO : fonction pour reinitialiser idle (clear le buffer et tout remettre, checkPeriod, ignorer erreur)
  switch(currentState) {
    case State::IDLE:
      if(data == PREAMBLE) {
        currentState = State::PREAMBLE;
      }
      break;
    case State::PREAMBLE:
      if(data == START_END) {
        currentState = State::START;
      } else {
        initState();
      }
      break;
    case State::START:
      currentState = State::HEADER_FLAG;
      break;
    case State::HEADER_FLAG:
      if(data + BASE_TRAME_SIZE > MAX_SIZE) {
        initState();
      } else {
        payloadSize = data;
        currentState = State::HEADER_SIZE;
      }
      break;
    case State::HEADER_SIZE:
      currentState = State::PAYLOAD;
      currentPayloadSize++;
      payload.push_back(data);
      break;
    case State::PAYLOAD:
      if(currentPayloadSize == payloadSize) {
        currentState = State::CRC;
        crcVal = data << 8;
      } else {
        currentPayloadSize++;
        payload.push_back(data);
      }
      break;
    case State::CRC:
      for(int i = 0; i < payloadSize; i++) {
        crc.add(payload[i]);
      }
      crcVal |= data & 0b11111111;
      if(crcVal == crc.getCRC()) {
        currentState = State::END;
      } else {
        initState();
      }
      break;
    case State::END:
        if(data == START_END) {
          // ajouter dans trame structure
          // print payload
          Serial.printf("Payload ");
          for(int i = 0; i < payload.size(); i++) {
            Serial.printf(" %d ", payload[i]);
          }
        }
        // Revenir à l'état initial
        printBuffer();
        initState();
        break;
  }
}

std::vector<uint8_t> createMessage(uint8_t *payloadArray, int size) {
  CRC16 crc;
  crc.add(payloadArray, size);
  uint16_t crcVal = crc.getCRC();

  std::vector<uint8_t> message{};
  message.reserve(MAX_SIZE);
  
  message.push_back(PREAMBLE);                    // preambule
  message.push_back(START_END);                   // start
  message.push_back(0b10000001);                  // header | Type + Flag
  message.push_back(size);                        // header | Size
  for(int i = 0; i < size; ++i) {                 // payload
    message.push_back(payloadArray[i]);
  }
  // message.push_back(0b00000000);
  // message.push_back(0b00000000);

  message.push_back((crcVal >> 8) & 0b11111111);  // CRC
  message.push_back(crcVal & 0b11111111);         // CRC
  message.push_back(START_END);

  return message;
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

    if(!checkPeriod && (periodElapse >= HALF_PERIOD*2 + TIME_BETWEEN_PERIODS - THRESHOLD_PERIOD) &&
       (periodElapse <= HALF_PERIOD*2 + TIME_BETWEEN_PERIODS + THRESHOLD_PERIOD)) {
        checkPeriod = true;
    }

    if(receivedBit) {
      receivedBit = false;
      rxVal = digitalRead(PIN_IN);
      

      if(buffer.size() != 0) {
        if(checkPeriod) {
          if(periodElapse >= HALF_PERIOD - THRESHOLD_PERIOD) { // one period has pass
            addBit(!rxVal);
          }
        } else {
          checkPeriod = true;
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
  const uint8_t size = 4;
  uint8_t payloadArray[size] = {
    0b11011101, 
    0b11011111,
    0b00011101,
    0b00000111,
  };
  
  std::vector<uint8_t> message = createMessage(payloadArray, size);

  for(int i = 0; i < 11; ++i) {
    sendByte(message[i]);
  } 

  for (;;) {
    vTaskDelay(xDelay);
  }
}


