#include <vector>


const int MAX_SIZE = 80;
const int BASE_TRAME_SIZE = 7;
const int PIN_OUT = 32; 
const int PIN_IN = 33;
const int RECEIVE_CORE = 0;
const int SEND_CORE = 1;
const int ZERO_MODE = 0;
const int ONE_MODE = 1;

std::vector<uint8_t> buffer;


// Define functions
void receiveRising();
void receiveFalling();  
void TaskReceive(void *pvParameters);
void TaskSend(void *pvParameters);

void setup() {
  // put your setup code here, to run once:
  // pinMode(PIN_OUT, OUTPUT);
  // pinMode(PIN_IN, INPUT);

  Serial.begin(115200);

  // Receive interrupts
  // attachInterrupt(digitalPinToInterrupt(PIN_IN), receiveRising, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(PIN_IN), receiveFalling, FALLING);

  // Receive Task
  xTaskCreate(TaskReceive, "Receive Trame", 2048, NULL, 2,  NULL);
  // Send Task
  xTaskCreate(TaskSend, "Send Trame", 2048, NULL, 3,  NULL);


}

void loop() {
  // put your main code here, to run repeatedly:

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
  };

  return trame;
}

void sendPulse(int mode) { // falling (1), rising (0)
  digitalWrite(PIN_OUT, mode);
  delayMicroseconds(100);

  digitalWrite(PIN_OUT, !mode);
  delayMicroseconds(100);
}

// void sendTrame(uint8_t *trame, int trame_size) {
//   for(int i = 0; i < trame_size; i++) {

//   }
// }

/*--------------------------------------------------*/
/*-------------------- Interrups -------------------*/
/*--------------------------------------------------*/

void receiveRising() {
  buffer.push_back(0);
  Serial.println("Rising : 0");
}

void receiveFalling() {
  buffer.push_back(1);
  Serial.println("Falling : 1");
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskReceive(void *pvParameters) { 
  // (void) pvParameters;
  while(true) {
    // Serial.println("TaskReceive");
    delay(1000);
  }
  
}

void TaskSend(void *pvParameters) {  
  while(true) {
    Serial.println("TaskSend");
    sendPulse(ONE_MODE);
    delay(1000);
    sendPulse(ZERO_MODE);
    delay(1000);
  }
}








