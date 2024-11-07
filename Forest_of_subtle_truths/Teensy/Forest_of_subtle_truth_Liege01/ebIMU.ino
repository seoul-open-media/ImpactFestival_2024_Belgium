void getImuData() {
  switch (state) {
    case STATE_INIT:
      imuMillis = millis();
      state = STATE_IMU_POLL;

      break;

    case STATE_IMU_POLL:
      i2cuartWrite(imuPoll);
      imuPollTime = millis(); // set time stamp
      state = STATE_IMU_POLL_WAIT;

      break;

    case STATE_IMU_POLL_WAIT:
      if (millis() > imuPollTime + imu_poll_wait_time) { // wait until timeout of 70ms.
        Serial.println("IMU poll timed out");
        state = STATE_INIT;
      } else {
        if (i2cuart.available() > 0) {   // we got data from EBIMU
          state = STATE_IMU_READ;
        }
      }

      break;

    case STATE_IMU_READ:
      // AudioNoInterrupts();
      EBimuAsciiParser(euler, 3);

      Roll = euler[0] - 90 + 13;
      Pitch = - euler[1] + 2;
      Head =  euler[2] + 180 - heading_offset; // heading_offset is set by neoKey and stored EEPROM address 2, 3
      // we added offset so set it to in the range of 0~359
      if (Head >= 360) Head -= 360;
      if (Head < 0) Head += 360;

      imuRoll = Roll;
      imuPitch = Pitch;
      imuHead = Head;
      

      // convert to radians
      radHead = imuHead * deg2rad;
      radPitch = imuPitch * deg2rad;
      radRoll = imuRoll * deg2rad;


      state = STATE_IMU_WAIT;
      // AudioInterrupts();

      break;


    case STATE_IMU_WAIT:
      if (millis() > imuMillis + IMU_POLL_INTERVAL) { //
        state = STATE_INIT;
      }

      break;


    default:
      break;

  }
}


void changeImuBaudrate() {
  // change the default baudrate of EBIMU to 921600
  long baudrate[8] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
  byte count = 0;
  for (int i = 7; i > -1; i--) {
    Serial.print("trying baudrate "); Serial.println(baudrate[i]);
    i2cuart.begin(baudrate[i]); //open connection
    delay(500);
    EBimuCommand("<sor0>"); // stop outputting data
    // delay(1000);

    // read all the buffer
    for (int i = 0; i < 1000; i++) {
      i2cuart.read(); // read all the buffer
    }
    // give other command
    EBimuCommand("<sor0>"); //start poliing mode and save it
    delay(500);

    if (i2cuart.read() == 60) { // see if it's answer correctly, ascii '<'
      // change it to 921600
      while (i2cuart.available())Serial.print(i2cuart.read());
      Serial.println();
      Serial.println("found correct baudrate");

      Serial.println("<sb8>");
      EBimuCommand("<sb8>");  // set baudrate to 921600
      delay(500);
      while (i2cuart.available())Serial.print(i2cuart.read());
      Serial.println();
      // it's ok to send other commands

      Serial.println("<start>");
      EBimuCommand("<start>");
      delay(500);
      while (i2cuart.available())Serial.print(i2cuart.read());
      Serial.println();

      Serial.println("start ebimu in 921600 baudrate");

      break;
    }
    count++;
  }
  // if it's 8 then all the baudrates failed to communicate
  if (count == 8) {
    displayData01();
    Serial.print("Check EBIMU!");
    while (1);
  }
  delay(1000);

  // i2cuart.begin(baudrate[i]);

}

void i2cuartWrite(String str) {
  String sb1 = str;
  byte buffer2[sb1.length() + 1];

  sb1.getBytes(buffer2, sb1.length() + 1);

  for (int i = 0; i < sb1.length() + 1; i++) {
    // AudioNoInterrupts();//////////////////////////////////////////////Important!
    i2cuart.write(buffer2[i]);
    // AudioInterrupts();
  }
}


int EBimuAsciiParser(float *item, int number_of_item)
{

  int n, i;
  int rbytes;
  char *addr;
  int result = 0;

  rbytes = i2cuart.available();

  for (n = 0; n < rbytes; n++)
  {
    // AudioNoInterrupts(); //////////////////////////////////////////////Important!
    sbuf[sbuf_cnt] = i2cuart.read();
    // AudioInterrupts();

    //    Serial.println(sbuf[sbuf_cnt],HEX);
    if (sbuf[sbuf_cnt] == 0x0a)
    {
      addr = strtok(sbuf, ",");
      for (i = 0; i < number_of_item; i++)
      {
        item[i] = atof(addr);

        addr = strtok(NULL, ",");

      }

      result = 1;
    }
    else if (sbuf[sbuf_cnt] == '*')
    { sbuf_cnt = -1;
    }

    sbuf_cnt++;
    if (sbuf_cnt >= SBUF_SIZE) sbuf_cnt = 0;

  }

  return result;

}

void EBimuCommand(String sb1) {
  byte buffer[sb1.length() + 1];

  sb1.getBytes(buffer, sb1.length() + 1);

  for (int i = 0; i < sb1.length() + 1; i++) {
    // AudioNoInterrupts();//////////////////////////////////////////////Important!
    i2cuart.write(buffer[i]);
    // AudioInterrupts();
  }
}

void displayIMUDataSetOffset() {

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.println("Press Enter to set offset");
  display.println("Press Exit to Menu");
  display.println("IMU Data:");
  display.print("Heading: ");
  display.println(imuHead);
  display.print("Pitch:   ");
  display.println(imuPitch);
  display.print("Roll:    ");
  display.println(imuRoll);
  display.print("Offset is ");
  display.println(heading_offset);
  display.display();
}
//
//void calibrate() {
//
//  switch (calibState) {
//    case CALIB_START:
//      EBimuCommand("<cmf>");
//      calibMillis = millis();
//      Serial.println("start calibrate");
//
//      calibState = CALIB_WAIT;
//      break;
//
//    case CALIB_WAIT:
//      if (millis() - calibMillis > 20000) {
//        EBimuCommand(">");
//        Serial.println("end calibrate");
//      }
//
//      calibState = CALIB_END;
//      break;
//
//    case CALIB_END:
//      calibState = CALIB_START;
//      serialState = 0;
//      break;
//
//    default:
//      break;
//  }
//
//  if (DEBUG)
//    Serial.println("done calibrating");
//}
