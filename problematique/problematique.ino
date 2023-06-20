#include <vector>

const int MAX_SIZE = 80;
const int BASE_TRAME_SIZE = 7;
const int PIN_OUT = 32; 
const int PIN_IN = 33;
const int RECEIVE_CORE = 0;
const int SEND_CORE = 1;
const int ZERO_MODE = 0;
const int ONE_MODE = 1;
const int HALF_PERIOD = 500;
const int THRESHOLD_PERIOD = 5;


std::vector<uint8_t> buffer;

int valIn;
int valOut;
unsigned long duration;

unsigned long timer;
unsigned long lastChangeTime;
bool checkPeriod;

// Define functions
void receiveRising();
// void receiveFalling();  
void receivePulse(); 
void TaskReceive(void *pvParameters);
void TaskSend(void *pvParameters);

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_OUT, OUTPUT);
  pinMode(PIN_IN, INPUT);

  Serial.begin(115200);

  // Receive interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_IN), receiveRising, RISING);
  // attachInterrupt(digitalPinToInterrupt(PIN_IN), receivePulse, CHANGE);

  // Receive Task
  xTaskCreate(TaskReceive, "Receive Trame", 2048, NULL, 2,  NULL);
  // Send Task
  xTaskCreate(TaskSend, "Send Trame", 2048, NULL, 3,  NULL);

  lastChangeTime = 0;
  checkPeriod = true;
}

void loop() {
  // put your main code here, to run repeatedly:
  // duration = pulseIn(PIN_IN);
  // Serial.printf("pulse duration %d ", duration);
  timer = micros();

  valIn = digitalRead(PIN_IN); 
  valOut = digitalRead(PIN_OUT); 

  // Serial.printf("in %d ", valIn);
  // Serial.printf("ou %d\n", valOut);

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

// void sendPulse(int mode) { // falling (1), rising (0)
//   digitalWrite(PIN_OUT, mode);
//   delayMicroseconds(100);

//   digitalWrite(PIN_OUT, !mode);
//   delayMicroseconds(100);
// }

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

void IRAM_ATTR receiveRising() {
  buffer.push_back(0);
  Serial.println("Rising : 0");
}

// void IRAM_ATTR receiveFalling() {
//   buffer.push_back(1);
//   Serial.println("Falling : 1");
// }

// void IRAM_ATTR receivePulse() { 
//   // Check if rising or falling
//   int rxVal = digitalRead(PIN_IN);
//   Serial.printf("\nChange in %d ", rxVal);
//   // if(checkPeriod) {
//   //   int diff = timer - lastChangeTime;
//   //   if((diff >= diff - THRESHOLD_PERIOD) && (diff <= diff + THRESHOLD_PERIOD)) { // one period has pass
//   //     checkPeriod = false;
//   //     // read pin et mettre dans buffer selon rising ou falling sachant que on est au 2eme change
//   //     // buffer.push_back(!rxVal); 
//   //   }
//   //   // si plus qu'une periode
//   //   //    push selon rising et falling
//   //   //    checkPeriod = true
//   // }



  

//   lastChangeTime = timer;
//   // Add bit to vector
//   // Notify task


// }

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
  const TickType_t xDelay = 500;
  while(true) {
    // Serial.println("TaskSend");
    sendZero();
    // delay(1000);
    // sendOne();
    vTaskDelay(xDelay);
    // delay(1000);
  }
}


