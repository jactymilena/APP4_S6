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
int diff;
int rxVal;
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

void loop() {

  // Serial.println("alllooooo");
  // put your main code here, to run repeatedly:
  // unsigned long duration = pulseIn(PIN_IN, HIGH);
  

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

// void sendTrame(uint8_t *trame, int trame_size) {
//   for(int i = 0; i < trame_size; i++) {

//   }
// }

/*--------------------------------------------------*/
/*-------------------- Interrups -------------------*/
/*--------------------------------------------------*/

// void IRAM_ATTR receivePulse() { 
//   rxVal = digitalRead(PIN_IN);
//   // Serial.printf("CHANGE %d\n", rxVal);

//   auto currentTime = micros();
//   diff = currentTime - lastChangeTime;

//   if(checkPeriod) {
    
//     // Serial.printf("Diff %d\n", diff);
//     if((diff >= HALF_PERIOD - THRESHOLD_PERIOD) && (diff <= HALF_PERIOD + THRESHOLD_PERIOD)) { // one period has pass
//     // Serial.println("half period");
//       checkPeriod = false;
//       // read pin et mettre dans buffer selon rising ou falling sachant que on est au 2eme change
//       // m.lock();
//       buffer.push_back(!rxVal); 
//       // m.unlock();
//       receivedBit = true;
//     // }
//     } else if(diff >= (HALF_PERIOD*2 - THRESHOLD_PERIOD)) { // two period has pass // 
//       // Serial.println("one period");
//       // si plus qu'une periode
//       //    push selon rising et falling
//       //    checkPeriod = true
//       checkPeriod = false;
//       // m.lock();
//       buffer.push_back(!rxVal); 
//       // m.unlock();
//       receivedBit = true;
//     }
//   } else {
//     checkPeriod = true;
//   }

//   lastChangeTime = currentTime;
//   // Add bit to vector
//   // Notify task
// }

void IRAM_ATTR receivePulse() {
  // buffer.push_back(digitalRead(PIN_IN));  
  receivedBit = true;
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskReceive(void *pvParameters) { 
  const TickType_t xDelay = 20;

  for (;;) {
    // delayMicroseconds(20);
    // vTaskDelay(xDelay);
    auto currentTime = micros();
    diff = currentTime - lastChangeTime;

    if(!checkPeriod && (diff >= HALF_PERIOD*2 + TIME_BETWEEN_PERIODS - THRESHOLD_PERIOD) && (diff <= HALF_PERIOD*2 + TIME_BETWEEN_PERIODS + THRESHOLD_PERIOD)) {
        checkPeriod = true;
        Serial.printf("if diff %d\n", diff);
    }

    // TODO : ajouter les 20 ticks (convertis au if en haut pour gerer le changement de bit)
    if(receivedBit) {
      receivedBit = false;
      
      rxVal = digitalRead(PIN_IN);
      

      Serial.printf("diff %6d rx  %d ", diff, rxVal);
      if(checkPeriod) {
        if(diff >= HALF_PERIOD - THRESHOLD_PERIOD) { // one period has pass
          checkPeriod = false;
          buffer.push_back(!rxVal); 
        }
        
        Serial.printf("buffer START");
        for(int i = 0; i < buffer.size(); i++) {
          Serial.printf(" %d", buffer[i]);
        }
        Serial.println(" END");
      } else {
        checkPeriod = true;
        Serial.println(" ");
      }


      // if(checkPeriod) {
  
      //   // if((diff >= HALF_PERIOD - THRESHOLD_PERIOD) && (diff <= HALF_PERIOD + THRESHOLD_PERIOD)) { // one period has pass
      //   //   checkPeriod = false;
      //   //   buffer.push_back(!rxVal); 
      //   //   Serial.printf("if 1 ");
      //   // } else if(diff >= (HALF_PERIOD*2 - THRESHOLD_PERIOD)) { // two period has pass // 
      //   //   checkPeriod = false;
      //   //   buffer.push_back(!rxVal); 
      //   //   Serial.printf("if 2  ");
      //   // } 

      //   if(diff >= HALF_PERIOD - THRESHOLD_PERIOD) { // one period has pass
      //     checkPeriod = false;
      //     buffer.push_back(!rxVal); 
      //     // Serial.printf("if ");
      //   }

      //   Serial.printf("buffer START");
      //   for(int i = 0; i < buffer.size(); i++) {
      //     Serial.printf(" %d", buffer[i]);
      //   }
      //   Serial.println(" END");
      // } else {
      //   checkPeriod = true;
      // }

      lastChangeTime = currentTime;
    }
  }
}

void TaskSend(void *pvParameters) {  
  // const TickType_t xDelay = 20;
  for(int i = 0; i < 5; i++) {
      sendOne();
      vTaskDelay(xDelay);
      // delayMicroseconds(20);
      // sendZero();
      // vTaskDelay(xDelay);
  }

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


