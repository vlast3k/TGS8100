void requestEvent() {
  receivedI2C = true;
  Wire.write(i2cdata, 15); // respond with message of 6 bytes
}

void receiveEvent(int howMany) {
  receivedI2C = true;
  uint8_t msg[20];
  for (int i=0; i < howMany; i++) { // loop through all but the last
    msg[i] = Wire.read(); 
  }
  if (msg[howMany - 1] != 42) {
    return;
  }
  switch (msg[0]) {
    case MSG_SET_TEMP_HUM: setTempHum(msg); break;
    case MSG_RESET: onMsgReset(); break;
    case MSG_READ: performRead = true; break;
  }
}

void setTempHum(uint8_t *msg) {
  int16_t itemp =  (msg[1] << 8) + msg[2];
  int16_t ihum  =   msg[3];
  cTemp = (float)itemp / 10;
  cHum  = (float)ihum;
  
}
void onMsgReset() {
  Serial << F("Resetting")<< endl;
  Serial.flush();
  clearEEPROM();
  asm volatile ("  jmp 0");  
}

