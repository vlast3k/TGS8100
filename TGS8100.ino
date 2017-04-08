#define PULSE 6
#define OUT A6
#include <Streaming.h>
#include <Wire.h>
#include <RunningAverage.h>
#include "LowPower.h"
#include <EEPROM.h>

#define MSG_SET_TEMP_HUM 0x1
#define MSG_RESET 0x2
#define ITERATIONS_CHECK 10
#define START_ITER 400
#define EE_MAX_R0_2B 0

boolean startedIterations = false;

uint16_t lastValue = 0;
RunningAverage raVoc(5);
uint8_t i2cdata[20];


//float cTemp = 20;
//float cHum  = 40;
float cTemp = 20;
float cHum  = 40;
uint16_t cMaxR0 = 120;
double cRs, cRsAdj;

bool receivedI2C = false;

String i2cLog = "";

uint32_t iterations = 0;
#define I2C_ADDR 8
void setup() {
  Serial.begin(115200l);
  pinMode(PULSE, OUTPUT);
  digitalWrite(PULSE, LOW);
  Serial << "Started TGS8100" << endl;
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  Wire.begin(I2C_ADDR);                // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event
  Wire.onReceive(receiveEvent);
  initEEPROM();
}

void loop() {
  onIteration();
  goToSleep();
  readADC();
  Serial << "Value: avg: " << raVoc.getAverage() << endl;
  delay(200);
}

void initEEPROM() {
  EEPROM.begin();
  int16_t tR0=0;
  EEPROM.get(EE_MAX_R0_2B, tR0);
  if (tR0 > -1) cMaxR0 = tR0;
}


void prepareData() {
  uint16_t value = raVoc.getAverage();
  double VIN = 3.3F  ;
  double vRfix = VIN* (double)value/1024;
  double RFIX = 220;
  //double R0 = 147; // in room air
  //double R0 = 185; //in outside
  double GRAD = 1.8;
  cRs = (3.0F - vRfix)/vRfix*RFIX;
  double adjFactor = 100.0F + getTempAdj(20, cTemp) + getHumAdj(40, cHum);
  Serial << "TempAdj: " << getTempAdj(20, cTemp) << endl;
  Serial << "HumAdj:  " << getHumAdj(40, cHum)   << endl;
  adjFactor /= (double)100;
  cRsAdj = cRs * adjFactor;
  double rsr0 = cRsAdj/cMaxR0;
  double ppm = pow(1.0F/rsr0, GRAD);
  uint16_t ppm100 = ppm*100;
  uint16_t rsu = cRs;
  uint16_t rsuadj = cRsAdj;
  Serial << "SSSSSSSSSSS";
  Serial.flush();
  Serial << "cTemp: " << cTemp;
  Serial.flush();
  Serial << ", chum: " << cHum << ", adjF:" << adjFactor << ", cRsAdj: " << cRsAdj << ", rs:" << cRs << endl;
  Serial.flush();
//  Serial << "ppm: " << ppm  << ", rs: " << rs << ", rsu: " << rsu << endl;
//  Serial << _HEX(rsu >> 8) << " "<< _HEX(rsu &0xFF) << endl;
  //delay(3000);
  uint8_t vcc = readVcc() / 100;
  Serial << "rsuadj: " << rsuadj << ", vcc: " << vcc << ", cMaxR0: " << cMaxR0 << endl;
  i2cdata[0] = (uint8_t)(value >> 8);
  i2cdata[1] = (uint8_t)(value & 0xFF);
  i2cdata[2] = (uint8_t)(rsu >> 8);
  i2cdata[3] = (uint8_t)(rsu & 0xFF);
  i2cdata[4] = (uint8_t)(ppm100 >> 8);
  i2cdata[5] = (uint8_t)(ppm100 & 0xFF);
  i2cdata[6] = (uint8_t)(rsuadj >> 8);
  i2cdata[7] = (uint8_t)(rsuadj & 0xFF);
  i2cdata[8] = (uint8_t)(vcc >> 8);
  i2cdata[9] = (uint8_t)(vcc & 0xFF);
  i2cdata[10] = (uint8_t)(cMaxR0 >> 8);
  i2cdata[11] = (uint8_t)(cMaxR0 & 0xFF);
  i2cdata[12] = 42;
  //Serial << data[0] << " " << data[1] << endl;  
}

void readADC() {
  uint32_t sum = 0;
  int i = 0;
  int values = 10;
  noInterrupts();
  digitalWrite(PULSE, HIGH);
  analogRead(OUT);
  analogRead(OUT);
  analogRead(OUT);
  for (; i < values; i++) {
    sum += analogRead(OUT); 
  }
  digitalWrite(PULSE, LOW);
  float v = (float)sum / values ;
  raVoc.addValue(v);  
  interrupts();  
  prepareData();

}


void goToSleep() {
  Serial << "Go To Sleep" << endl;
  Serial.flush();
  //delay(1000);
  LowPower.powerDown(SLEEP_8S, ADC_ON, BOD_OFF);   
  //Serial << "w " << millis() << endl;
  TWCR = bit(TWEN) | bit(TWIE) | bit(TWEA) | bit(TWINT);
  Wire.begin(I2C_ADDR);
  delay(200);
  Serial <<"i2cLog:" << i2cLog << endl;
}

void onIteration() {
  if (!receivedI2C) iterations++;
//  Serial << "stariter:" << startedIterations << ", iter: " << iterations << endl;
  receivedI2C = false;  
  if (!startedIterations) {
  //Serial << "starit000er:" << startedIterations << ", iter: " << iterations << endl;
    if (iterations == START_ITER) {
      startedIterations = true;
      iterations = 0;
    }
    return;
  } else {
    //Serial << "starit111er:" << startedIterations << ", iter: " << iterations << endl;
    if (iterations > ITERATIONS_CHECK) {
      iterations = 0;
      if (cMaxR0 < (uint16_t)cRsAdj) {
        cMaxR0 = cRsAdj;
        EEPROM.put(EE_MAX_R0_2B, cMaxR0);
      }    
    }
  }
}


void clearEEPROM() {
  for (int i=0; i < 300; i++) EEPROM.write(i, 255);
}

#define TEMP_ADJ_0_10  2.0F
#define TEMP_ADJ_10_20 1.6F
#define TEMP_ADJ_20_30 1.3F
#define TEMP_ADJ_30_40 0.9F
double getTempAdj(double from, double to) {
  bool inv = false;
  if (from > to) {
    double t = to;
    to = from;
    from = t;
    inv = true;
  }
  double adj = 0;
  if (from < 10) adj += max(0, (min(10.0f, to) - max(from, 0.0F) ) * TEMP_ADJ_0_10);
  if (from < 20) adj += max(0, (min(20.0f, to) - max(from, 10.0F)) * TEMP_ADJ_10_20);
  if (from < 30) adj += max(0, (min(30.0f, to) - max(from, 20.0F)) * TEMP_ADJ_20_30);
  if (from < 40) adj += max(0, (min(40.0f, to) - max(from, 30.0F)) * TEMP_ADJ_30_40);
  
  if (inv) adj = -adj;
  return adj;
}

#define HUM_ADJ_0_20 1.0F
#define HUM_ADJ_20_40 0.9F
#define HUM_ADJ_40_65 0.5F
#define HUM_ADJ_65_85 0.3F
double getHumAdj(double from, double to) {
  bool inv = false;
  if (from > to) {
    double t = to;
    to = from;
    from = t;
    inv = true;
  }
  double adj = 0;
  Serial  << "H adj1: " << adj << endl;
  if (from < 20) adj += max(0, (min(20.0f, to) - max(from, 0.0F)) * HUM_ADJ_0_20);
  Serial  << "H adj1: " << adj << endl;
  if (from < 40) adj += max(0, (min(40.0f, to) - max(from, 20.0F)) * HUM_ADJ_20_40);
  Serial  << "H adj2: " << adj << endl;
  if (from < 65) adj += max(0, (min(65.0f, to) - max(from, 40.0F)) * HUM_ADJ_40_65);
  Serial  << "H adj3: " << adj << endl;
  if (from < 85) adj += max(0, (min(85.0f, to) - max(from, 65.0F)) * HUM_ADJ_65_85);
  Serial  << "H adj4: " << adj << endl;
  
  if (inv) adj = -adj;
  return adj;
}

long readVcc() { 
  long result; // Read 1.1V reference against AVcc 
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1); 
  delay(2); // Wait for Vref to settle 
  ADCSRA |= _BV(ADSC); // Convert 
  while (bit_is_set(ADCSRA,ADSC)); 
  result = ADCL; 
  result |= ADCH<<8; 
  result = 1126400L / result; // Back-calculate AVcc in mV 
  return result; 
}


