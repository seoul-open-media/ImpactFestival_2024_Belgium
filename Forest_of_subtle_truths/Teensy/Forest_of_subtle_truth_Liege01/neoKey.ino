void neoKey() {
  //Serial.println("neoKey");
  for (uint8_t i = 0; i < 4; i++) {
    debouncers[i].update();
  }
  if (debouncers[0].rose()) {
    neoKey_num++;
    Serial.print("1 preseed. neoKey_num is "); Serial.println(neoKey_num);
    update_screen = true;
  }
  if (debouncers[1].rose()) {
    neoKey_num--;
    Serial.print("2 preseed. neoKey_num is "); Serial.println(neoKey_num);
    update_screen = true;
  }
  if (debouncers[2].rose()) {
    neoKey_enter = debouncers[2].rose();
    update_screen = true;
  }
  if (debouncers[3].rose()) {
    neoKey_exit = debouncers[3].rose();
    update_screen = true;
  }
}

void neoKeyPulseAll() {
  // Pulse all the LEDs on to show we're working
  for (uint16_t i = 0; i < neokey.pixels.numPixels(); i++) {
    neokey.pixels.setPixelColor(i, 0x808080); // make each LED white
    neokey.pixels.show();
    delay(50);
  }
  for (uint16_t i = 0; i < neokey.pixels.numPixels(); i++) {
    neokey.pixels.setPixelColor(i, 0x000000);
    neokey.pixels.show();
    delay(50);
  }

}

void neoKeyConfigSettings() {
  int row = 0;
  while (cfg_mode == true) {
    //Serial.println("cfg_mode");
    neoKey();

    if ( screen == SCREEN_MAIN_CFG_MENU) {

      byte num_menu = 4; // total number of menu
      char* menu[] = {"Set Headphone Address", "IMU Calibration", "Set IMU Offset", "Exit"};

      if (neoKey_num < 0) neoKey_num = 0;
      if (neoKey_num > (num_menu - 1)) neoKey_num = (num_menu - 1);
      row = (num_menu - 1) - neoKey_num; // arrow down for the next line
      if (update_screen == true) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("MENU");
        display.setTextSize(1);
        display.println();
        display.setCursor(0, 20);
        //delay(100);

        // Scroll menu with Highlight
        for (int i = 0; i < num_menu ; i++) {

          if (row == i) {
            display.setTextColor(BLACK, WHITE); // 'inverted' text
          } else {
            display.setTextColor(WHITE);
          }
          display.println(menu[i]);

        }

        //display.print(row);  // for debug

        display.display();
        update_screen = false;
      }
      // selection of menu
      if (neoKey_enter == true) {

        switch (row) {

          case (0):
            //
            screen = SCREEN_SET_HEADPHONES_ADDRESS;
            neoKey_num = my_address;
            update_screen = true;
            neoKey_enter = false;
            break;

          case (1):
            //
            screen = SCREEN_IMU_CALIBRATION;
            neoKey_num = 0;
            update_screen = true;
            neoKey_enter = false;
            break;

          case (2):
            //
            screen = SCREEN_SET_IMU_OFFSET;
            neoKey_num = 0;
            update_screen = true;
            neoKey_enter = false;
            break;

          case (3):
            //Exit
            cfg_mode = false;
            update_screen = true;
            neoKey_enter = false;
            break;

          default:
            break;

        }
        // previous_screen = setting_screen;
      }
    }

    if ( screen == SCREEN_SET_HEADPHONES_ADDRESS) {
      if (update_screen == true) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("Set Headphone Address");
        display.setTextSize(1);
        display.println();
        display.setCursor(0, 20);
        display.println(neoKey_num);
        display.display();
        update_screen = false;
      }
      //delay(100);
      // store data to EEPROM
      if (neoKey_enter) {
        EEPROM.write( EP_ADDR_MY_ADDRESS , neoKey_num);
        my_address = EEPROM.read(EP_ADDR_MY_ADDRESS);
        display.clearDisplay();
        display.setCursor(0, 20);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("Stored in EEPROM!");
        display.print("my_address is ");
        display.println(my_address);
        display.display();
        delay(2000);
        screen = SCREEN_MAIN_CFG_MENU;
        //initialize
        neoKey_num = num_menu - 1;
        neoKey_enter = false;
        neoKey_exit = false;
        update_screen = true;
      }
      if (neoKey_exit) {
        screen = SCREEN_MAIN_CFG_MENU;
        //initialize
        neoKey_num = num_menu - 1;
        neoKey_enter = false;
        neoKey_exit = false;
        update_screen = true;
      }
    }

    if ( screen == SCREEN_IMU_CALIBRATION) {
      if (update_screen == true) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("IMU Calibration");
        display.setTextSize(1);
        display.println();
        // display.setCursor(0, 20);
        display.println("Press Enter to start!");
        //display.println();
        display.println("Keep the distance between each side of the headphones about 20cm away from magnetic field.");
        display.println("Away from magnetic field.");
        display.display();
        update_screen = true;
      }
      //delay(100);

      if (neoKey_enter) {
        if (update_screen == true) {
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println("IMU Calibration");
          display.setTextSize(1);
          display.println();
          display.setCursor(0, 20);
          display.println("In progress!");
          display.println("");
          display.println("Press Exit to finish.");
          display.display();
          //update_screen = false;

        }
        //delay(100);
        neoKey_exit = false;
        EBimuCommand("<cmf>");

        while (!neoKey_exit) {
          Serial.println(i2cuart.read());
          neoKey();
          //  neoKeyPulseAll();
        }

        EBimuCommand(">");

        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("IMU Calibration");
        display.setTextSize(1);
        display.println();
        display.setCursor(0, 20);

        delay(1000);
        if (i2cuart.read() == 60) {
          display.println("Calibration finished!");
          display.display();
          delay(1000);
        } else {
          display.println("didn't get <ok> from IMU");
          display.println("Error!");
          display.display();
          delay(10000);
        }

        screen = SCREEN_MAIN_CFG_MENU;
        //initialize
        neoKey_num = num_menu - 1;
        neoKey_enter = false;
        neoKey_exit = false;
        update_screen = true;
      }
      if (neoKey_exit) {
        screen = SCREEN_MAIN_CFG_MENU;
        //initialize
        neoKey_num = num_menu - 1;
        neoKey_enter = false;
        neoKey_exit = false;
        update_screen = true;
      }


    }

    if (screen == SCREEN_SET_IMU_OFFSET) {
      imu_poll_wait_time = 200;
      heading_offset = 0; // reset offset!!!

      while (!neoKey_exit) {
        neoKey();
        getImuData();
        //displayEverySecIMU(1);// scaled and smoothed value
        displayIMUDataSetOffset();

        delay (50);
        if (neoKey_enter) {


          // imuHead 0 ~ 359
          heading_offset = imuHead;
          // store in EEPROM address 1(MSB), 2(LSB)
          // devision factor is 250
          EEPROM.write( EP_ADDR_HEAD_OFFSET_MSB , byte(int(heading_offset) / 250));
          EEPROM.write( EP_ADDR_HEAD_OFFSET_LSB , byte(int(heading_offset) % 250));
          heading_offset = EEPROM.read(EP_ADDR_HEAD_OFFSET_MSB)*250 + EEPROM.read(EP_ADDR_HEAD_OFFSET_LSB);
          display.clearDisplay();
          display.setCursor(0, 20);
          display.setTextSize(1);
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println("Stored in EEPROM!");
          display.print("Offset: ");
          display.println(heading_offset);
          display.display();
          Serial.print("Offset: ");
          Serial.println(heading_offset);
          delay(2000);
          //initialize
          neoKey_num = num_menu - 1;
          neoKey_enter = false;
          neoKey_exit = false;
          update_screen = true;
          imu_poll_wait_time = 70;////////////// default value was 70
          break;
        }


      }
      screen = SCREEN_MAIN_CFG_MENU;
      //initialize
      neoKey_num = num_menu - 1;
      neoKey_enter = false;
      neoKey_exit = false;
      update_screen = true;

    }
  }
}
