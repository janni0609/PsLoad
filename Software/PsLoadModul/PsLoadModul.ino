
unsigned long myTime;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//DAC
#include <SPI.h>
#include <DAC7565.h>
DAC dac(PIN_PA5, PIN_PA7, PIN_PA4, PIN_PA1, PIN_PA3);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//ADC

#include <SparkFun_ADS1219.h>
SfeADS1219ArdI2C myADC;
const uint8_t ADCReset = PIN_PC5;
//const uint8_t interruptPin = PIN_PC4;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Timer Interrupt

#define USING_FULL_CLOCK      true

// Try to use RTC, TCA0 or TCD0 for millis()
#define USE_TIMER_0           true          // Check if used by millis(), Servo or tone()
#define USE_TIMER_1           false         // Check if used by millis(), Servo or tone()

#if USE_TIMER_0
  #define CurrentTimer   ITimer0
#elif USE_TIMER_1
  #define CurrentTimer   ITimer1
#else
  #error You must select one Timer  
#endif

#include "ATtiny_TimerInterrupt.h"

//#define TIMER1_INTERVAL_MS        25L

volatile bool prozessStuff = 0;


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
const uint8_t SenseSW = PIN_PC3;
const uint8_t ON = PIN_PC0;
const uint8_t OnPhoto = PIN_PC1;

const uint8_t DataOut1 = PIN_PB4;       //Send Data
const uint8_t DataOut2 = PIN_PB5;       //Error flag

const uint8_t DataIn1 = PIN_PB6;
const uint8_t DataIn2 = PIN_PB7;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Interpollation Cal
#include "InterpolationLib.h"

double VoltSetPoints[10] =  {0.03f,  0.05f, 0.1f,  1.0f, 5.0f, 10.0f, 15.0f, 20.0f, 24.0f, 25.0f};
double VoltDacOffsets[10];
double VoltAdcOffsets[10];


double AmpSetPoints[] =     {0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
double AmpDacPOffsets[10];
double AmpDacNOffsets[10];

double AmpAdcSetPoints[] =     {-5.0,     -4.9,     -4.5,    -4.0,    -1.0,    -0.5,    -0.1,    -0.05,    -0.005,     0.0,     0.0,     0.005,     0.05,    0.1,    0.5,    1.0,    4.0,    4.5,    4.9,     5.0};
double AmpAdcOffsets[20];
  
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Temp
const uint8_t Temper1 = PIN_PC2;
const uint8_t Temper2 = PIN_PA2;

const uint8_t OTP = 60;           // Over temp Protection
bool OTP_flag = 0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Globals for Sending
  float Volts, Amps, Temp1, Temp2;
  volatile bool ReadSerial = 0;

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//Eeprom

#include <EEPROM.h>


void setup()
{
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //Timer Interrupt

  CurrentTimer.init();
  
  CurrentTimer.attachInterruptInterval(25L , TimerHandler1);
  
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //Temp
  pinMode(Temper1, INPUT);
  pinMode(Temper2, INPUT);
  analogReadResolution(12);
  analogReference(INTERNAL2V048);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //Serial
  Serial.begin(1000000);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // Initialize the DAC
  pinMode(PIN_PA6, OUTPUT);
  digitalWrite(PIN_PA6, LOW);
  delay(100);

  dac.init();
  delay(100);

  // Set DAC reference to be powered up, VREF = 2.5V
  // this mean that vout will be able to go from 0V to 2.5V (full scale)
  dac.setReference(DAC_REFERENCE_ALWAYS_POWERED_UP);
  delay(10);

  // Set all outputs to 1V
  dac.writeChannel(DAC_CHANNEL_ALL, 0);
  Serial.println("DAC Setup");

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //ADC
  pinMode(ADCReset, OUTPUT);
  digitalWrite(ADCReset, HIGH);

  Wire.begin();
  Wire.setClock(400000);
  delay(10);
  

  while (myADC.begin() == false)
  {
    Serial.println("ADC failed to begin. Please check your wiring! Retrying...");
    delay(1000);
  }
  delay(10);
  myADC.setVoltageReference(ADS1219_VREF_EXTERNAL);
  myADC.setDataRate(ADS1219_DATA_RATE_90SPS);
  myADC.setConversionMode(ADS1219_CONVERSION_SINGLE_SHOT);
  
  Serial.println("ADC Setup");
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  pinMode(SenseSW, OUTPUT);
  digitalWrite(SenseSW, LOW);

  pinMode(ON, OUTPUT);
  digitalWrite(ON, LOW);

  pinMode(OnPhoto, OUTPUT);
  digitalWrite(OnPhoto, LOW);

  pinMode(DataIn2, INPUT);
  pinMode(DataIn1, INPUT);
  attachInterrupt(digitalPinToInterrupt(DataIn1),  ReadData, FALLING);

  pinMode(DataOut1, OUTPUT);
  pinMode(DataOut2, OUTPUT);
  digitalWrite(DataOut1, HIGH);
  digitalWrite(DataOut2, LOW);
  

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  
  //EEPROM.put(100, VoltAdcOffsets);
  //EEPROM.put(0, VoltDacOffsets);
  
  delay(10);

  EEPROM.get(0, VoltDacOffsets);
  EEPROM.get(40, VoltAdcOffsets);
  EEPROM.get(80, AmpDacPOffsets);
  EEPROM.get(120, AmpDacNOffsets);
  EEPROM.get(160, AmpAdcOffsets);
  //SetZero();
}

void TimerHandler1()    //ISR
{
  prozessStuff = 1;
}

void ReadData(void)     //ISR
{
  ReadSerial = 1;
}

void loop()
{

  

  if (digitalRead(DataIn2) && prozessStuff == 1){
    if (ReadSerial == 1) CheckSerialRx();
    ReadADCs();
    ReadTemps();
    SendData();
    
    //SendArray('y', VoltDacOffsets);
    //delay(10);
    //SumUp(VoltDacOffsets);
    prozessStuff = 0;
    //delay(25);
  }
}


void CheckSerialRx(void)
{
  while (Serial.available() > 0) {
    float num = 0.0;
    char first = Serial.read();
    if (first == 'v') {
      num = Serial.parseFloat();
      num += float(Interpolation::Linear(VoltSetPoints, VoltDacOffsets, 10, double(num), false));                     // cal
      num = constrain(num, 0.0, 25.0);
      num = num * 2621.4;                               //input  0-25 output 0-65535
      dac.writeChannel(DAC_CHANNEL_A, uint16_t(num));
      //Serial.println(num);
    }
    if (first == 'i') {
      num = Serial.parseFloat();
      num += float(Interpolation::Linear(AmpSetPoints, AmpDacPOffsets, 10, double(num), false));                     // cal
      num = constrain(num, 0.0, 5.0);
      num = num * 13107.0;                              //input  0-5 output 0-65535
      dac.writeChannel(DAC_CHANNEL_B, uint16_t(num));
      //Serial.println(num);
    }
    if (first == 's') {
      num = Serial.parseFloat();
      num += float(Interpolation::Linear(AmpSetPoints, AmpDacNOffsets, 10, double(num), false));                     // cal
      num = constrain(num, 0.0, 5.0);
      num = num * 13107.0;                               //input  0-5 output 0-65535
      dac.writeChannel(DAC_CHANNEL_C, uint16_t(num));
      //Serial.println(num);
    }
    if (first == 'o') {                                  //on
      digitalWrite(OnPhoto, HIGH);
      delayMicroseconds(1200);                           //mos turn on delay
      digitalWrite(ON, HIGH);
      //Serial.println("output ON");
    }
    if (first == 'f') {                                  //off
      digitalWrite(OnPhoto, LOW);
      delayMicroseconds(300);                            //mos turn off delay
      digitalWrite(ON, LOW);
      //Serial.println("output OFF");
    }
    if (first == 'h') {                                  //Sense internal
      digitalWrite(SenseSW, LOW);
      //Serial.println("Sense internal");
    }
    if (first == 'u') {                                  //Sense external
      digitalWrite(SenseSW, HIGH);
      //Serial.println("Sense external");
    }
    if (first == 'r') {                                  //Raw Data in
      uint16_t Raw = Serial.parseInt();
      
      dac.writeChannel(DAC_CHANNEL_A, Raw);
      //Serial.println(num);
    }
    if (first == 'X') {
      //EEPROM.put(0, StorCalConst);
      Zero(VoltDacOffsets, 10);
      Zero(VoltAdcOffsets, 10);
    }
    if (first == 'x') {
      //EEPROM.put(0, StorCalConst);
      Zero(AmpDacPOffsets, 10);
      Zero(AmpDacNOffsets, 10);
      Zero(AmpAdcOffsets, 20);
    }
    if (first == 'Y') {
      SendArray('y', VoltDacOffsets);
    }
    if (first == 'L'){
      uint8_t n = Serial.parseInt();
      float flt = Serial.parseFloat();
      VoltDacOffsets[n] = flt;
    }
    if (first == 'K'){
      uint8_t n = Serial.parseInt();
      float flt = Serial.parseFloat();
      VoltAdcOffsets[n] = flt;
    }
    if (first == 'I'){
      uint8_t n = Serial.parseInt();
      float flt = Serial.parseFloat();
      AmpDacPOffsets[n] = flt;
    }
    if (first == 'O'){
      uint8_t n = Serial.parseInt();
      float flt = Serial.parseFloat();
      AmpDacNOffsets[n] = flt;
    }
    if (first == 'P'){
      uint8_t n = Serial.parseInt();
      float flt = Serial.parseFloat();
      AmpAdcOffsets[n] = flt;
    }
    if (first == 'S') {                                  //Cal
      EEPROM.put(0, VoltDacOffsets);
      EEPROM.put(40, VoltAdcOffsets);
      EEPROM.put(80, AmpDacPOffsets);
      EEPROM.put(120, AmpDacNOffsets);
      EEPROM.put(160, AmpAdcOffsets);
    }
  }
  ReadSerial = 0;
}


void Zero(double * array, int size){
  for (uint8_t n = 0; n < size; n++){
    array[n] = 0.0;
  }
}

void SendArray(char prefix, double array[]){
  
  //Serial.print(prefix);
  for (uint8_t n = 0; n <= 9; n++){
    digitalWrite(DataOut1,LOW);
    Serial.print('y');
    Serial.print(array[n], 7);
    digitalWrite(DataOut1,HIGH);
  }
  
}


void ReadADCs(void)
{
  myADC.setInputMultiplexer(ADS1219_CONFIG_MUX_DIFF_P0_N1);
  if (myADC.startSync()) // Start a single-shot conversion. This will return true on success.
  {
    while (myADC.dataReady() == false) // Check if the conversion is complete. This will return true if data is ready.
    {
      delay(1); // The conversion is not complete. Wait a little to avoid pounding the I2C bus.
    }

    myADC.readConversion(); // Read the conversion result from the ADC. Store it internally.
    float ADCreading = myADC.getConversionMillivolts(2500.0); // Convert to millivolts.
    ADCreading = 1250.0 - ADCreading;
    ADCreading = ADCreading * 4.0;
    Amps = ADCreading/1000.0;
    Amps += Interpolation::Linear(AmpAdcSetPoints, AmpAdcOffsets, 20, Amps, true);
    Amps = Amps - Volts / 110000.0;
    //Serial.print("mAmps: ");
    //Serial.print(ADCreading, 3);
  }

  myADC.setInputMultiplexer(ADS1219_CONFIG_MUX_DIFF_P2_N3);
  if (myADC.startSync()) // Start a single-shot conversion. This will return true on success.
  {
    while (myADC.dataReady() == false) // Check if the conversion is complete. This will return true if data is ready.
    {
      delay(1); // The conversion is not complete. Wait a little to avoid pounding the I2C bus.
    }

    myADC.readConversion(); // Read the conversion result from the ADC. Store it internally.
    float milliVolts = myADC.getConversionMillivolts(2500.0); // Convert to millivolts.
    Volts = milliVolts/100.0;
    Volts += Interpolation::Linear(VoltSetPoints, VoltAdcOffsets, 10, Volts, true);
    Volts = Volts;
    //Serial.print("   Volt: ");
    //Serial.print(milliVolts / 100.0, 6);
  }
}

void SendData(void)
{
  digitalWrite(DataOut1,LOW);
  Serial.print("V");
  Serial.print(Volts,6);
  Serial.print("A");
  Serial.print(Amps,6);
  Serial.print("n");
  Serial.print(Temp1,1);
  Serial.print("m");
  Serial.print(Temp2,1);
  Serial.println("l");
  digitalWrite(DataOut1,HIGH);
}

void ReadTemps(void)
{
  float tempsensor = analogRead(Temper1);
  tempsensor = tempsensor * 0.0005;             //convert to volt
  tempsensor = tempsensor - 0.5;
  tempsensor = tempsensor/0.01;                 //convert to Celsius
  Temp1 = tempsensor;


  tempsensor = analogRead(Temper2);
  tempsensor = tempsensor * 0.0005;             //convert to volt
  tempsensor = tempsensor - 0.5;
  tempsensor = tempsensor/0.01;                 //convert to Celsius
  Temp2 = tempsensor;

  float maxTemp;

  if (Temp1 > Temp2) maxTemp = Temp1;
  else maxTemp = Temp2;

  if (maxTemp >= OTP && OTP_flag == 0){
    OTP_flag = 1;
    digitalWrite(OnPhoto, LOW);
    delayMicroseconds(300);                           //mos turn off delay
    digitalWrite(ON, LOW);
    digitalWrite(DataOut2, HIGH);                     //Raise Error Flag
  }else if (maxTemp < OTP && OTP_flag == 1){
    OTP_flag = 0;
    digitalWrite(DataOut2, LOW);                      //Reset Error Flag
    //digitalWrite(OnPhoto, HIGH);
    //delayMicroseconds(1200);                        //mos turn on delay
    //digitalWrite(ON, HIGH);
  }

}
