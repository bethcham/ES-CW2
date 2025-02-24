#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

volatile uint32_t currentStepSize = 0;

struct {
  std::bitset<32> inputs;  
  } sysState;

HardwareTimer sampleTimer(TIM1);

constexpr float baseFreq = 440.0; // frequency for A
constexpr float sampleFreq = 22000.0; // fs

constexpr uint32_t stepSizes[12] = {
  static_cast<uint32_t>((baseFreq * pow(2.0, -9.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // C
  static_cast<uint32_t>((baseFreq * pow(2.0, -8.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // C#
  static_cast<uint32_t>((baseFreq * pow(2.0, -7.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // D
  static_cast<uint32_t>((baseFreq * pow(2.0, -6.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // D#
  static_cast<uint32_t>((baseFreq * pow(2.0, -5.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // E
  static_cast<uint32_t>((baseFreq * pow(2.0, -4.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // F
  static_cast<uint32_t>((baseFreq * pow(2.0, -3.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // F#
  static_cast<uint32_t>((baseFreq * pow(2.0, -2.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // G
  static_cast<uint32_t>((baseFreq * pow(2.0, -1.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq), // G#
  static_cast<uint32_t>((baseFreq * (uint64_t(1) << 32)) / sampleFreq),                         // A
  static_cast<uint32_t>((baseFreq * pow(2.0, 1.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq),  // A#
  static_cast<uint32_t>((baseFreq * pow(2.0, 2.0 / 12.0) * (uint64_t(1) << 32)) / sampleFreq)   // B
};

const char* notes[12] = {
  "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
};

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);

  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);

  digitalWrite(REN_PIN, HIGH);
}

void sampleISR(){
  static uint32_t phaseAcc = 0;
  uint32_t localCurrentStepSize = __atomic_load_n(&currentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localCurrentStepSize;

  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    sysState.inputs.reset();

    for (uint8_t i = 0; i < 3; i++){
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> cols = readCols();
      sysState.inputs |= (cols.to_ulong() << (i*4));
    }

    bool keyPressed = false;
    uint32_t localCurrentStepSize = 0;

    for (uint8_t i = 0; i < 12; i++) {
      if (!sysState.inputs[i]) {
        localCurrentStepSize = stepSizes[i];
        keyPressed = true;
        u8g2.setCursor(2,30);
        u8g2.print(notes[i]);
        break; 
      }
    }

    if (!keyPressed) {
      localCurrentStepSize = 0; 
      u8g2.setCursor(2,30);
      u8g2.print("-");
    }

    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
    
    u8g2.setCursor(2,20);
    u8g2.print(sysState.inputs.to_ulong(), HEX);

    u8g2.sendBuffer();          // transfer internal memory to the display
    
    digitalToggle(LED_BUILTIN);
  }
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  TaskHandle_t scanKeysHandle = NULL;

  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,
    "displayUpdate",
    256,
    NULL,
    1,
    &displayUpdateHandle
  );

  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:  
}