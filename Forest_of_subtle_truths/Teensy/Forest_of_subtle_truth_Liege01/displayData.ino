void displayData00() {

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.print("Unable to access the SD card");
  display.display();
}
void displayData01() {

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Changing baudrate of IMU");
  display.println("Reboot afterwards!");
  display.display();
}

void displayData02() {

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("SC16IS750 not found");
  display.display();
}

void displayData04() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("MixHeadphones ");
  display.println(my_address);
  display.println();
  display.println("D from Anchor:");
  for (int i = 0; i < 3; i++) {
    display.print(distance_result[i] / 10.0); // DISTANCE IN METER
    display.print(", ");
  }
  display.println();
  display.println();
  display.print("elapsed time:");
  display.print(elapsed_time);
  display.print("ms.");
  display.display();
}

void displayData05() {
  int reading;
  float bat_voltage;
  char report[80];

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("MixHeadphones ");
  display.println(my_address);
  display.print("numZone: "); display.println(numZone);

  display.println("D from Anchor:");
  for (int i = 0; i < 3; i++) {
    display.print(distance_result[i] / 10.0); // DISTANCE IN METER
    display.print(", ");
  }
  display.println();
  display.print("X= ");
  display.println(x);
  display.print("Y= ");
  display.println(y);
  //  display.println();
  reading = analogRead(A0);
  bat_voltage = 3.3 * 2 * reading / 1024.0;
  //  display.print("time= ");
  //  display.println(int(millis() / 1000));

  display.print("V-Bat= ");
  display.print(bat_voltage);
  display.print("   ");
  //int bat_percent = int(((bat_voltage - 2.2) / (4.14 - 2.2)) * 100);
  int bat_percent = int(((bat_voltage - 3.356) / (4.04 - 3.356)) * 100);
  if (bat_percent > 100)bat_percent = 100;
  if (bat_percent < 0)bat_percent = 0;
  display.setTextSize(1);
  display.print(bat_percent);
  display.println("%");
  
  
  
  if (bat_voltage < 3.45) {
    display.clearDisplay();
    display.setTextSize(2);
    display.println("BATTERY LOW");
  }
  display.display();
}
// 1 or more anchor distancing error

void displayData06() {
  float distance_in_meter;
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("MixHeadphones ");
  display.println(my_address);
  //display.println();

  display.println("D from Anchor:");
  for (int i = 0; i < 3; i++) {
    distance_in_meter = distance_result[i] / 10.0;
    if ( distance_in_meter < max_distance[i] && distance_in_meter > min_distance[i]) {
      display.print(distance_in_meter); // DISTANCE IN METER
    } else {
      display.print("E");
    }

    display.print(", ");
  }

  display.print("elapsed time:");
  display.println();
  display.print("elapsed time:");
  display.print(elapsed_time);
  display.print("ms.");
  display.display();
}


void displayData() {
  int reading;
  float bat_voltage;
  char report[80];


  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print("Heaphones v.");
  display.println(VERSION);

  display.println();

  display.print("Software v. ");
  display.println(SOFT_VERSION);
  display.println();
  display.print("Hedphone ID: ");
  display.println(MY_ADDRESS);
  display.println();

  reading = analogRead(A0);
  bat_voltage = 3.3 * 2 * reading / 1024.0;
  //  display.print("time= ");
  //  display.println(int(millis() / 1000));

  display.print("V-Bat= ");
  display.print(bat_voltage);
  display.print("   ");
  //int bat_percent = int(((bat_voltage - 2.2) / (4.14 - 2.2)) * 100);
  int bat_percent = int(((bat_voltage - 3.356) / (4.04 - 3.356)) * 100);
  if (bat_percent > 100)bat_percent = 100;
  if (bat_percent < 0)bat_percent = 0;
  display.setTextSize(1);
  display.print(bat_percent);
  display.println("%");
  display.setTextSize(1);
  if (bat_voltage < 3.45) display.println("BATTERY LOW");
  display.display();
}

void displayIMUData() {

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.println("Sensor : EBIMU");
  display.println();
  display.println("IMU Data:");
  display.print("Heading: ");
  display.println(imuHead);
  display.print("Pitch:   ");
  display.println(imuPitch);
  display.print("Roll:    ");
  display.println(imuRoll);
  display.display();
}



void displayDataInfo() {
  int reading;
  float bat_voltage;
  char report[80];


  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);

  display.print("Heaphones v.");
  display.println(VERSION);

  display.println();

  display.print("Software v. ");
  display.println(SOFT_VERSION);
  display.println();
  display.print("Hedphone ID: ");
  display.println(my_address);
  display.println();

  reading = analogRead(A0);
  bat_voltage = 3.3 * 2 * reading / 1024.0;
  //  display.print("time= ");
  //  display.println(int(millis() / 1000));

  display.print("V-Bat= ");
  display.print(bat_voltage);
  display.print("   ");
  //int bat_percent = int(((bat_voltage - 2.2) / (4.14 - 2.2)) * 100);
  int bat_percent = int(((bat_voltage - 3.356) / (4.04 - 3.356)) * 100);
  if (bat_percent > 100)bat_percent = 100;
  if (bat_percent < 0)bat_percent = 0;
  display.setTextSize(1);
  display.print(bat_percent);
  display.println("%");
  display.setTextSize(1);
  if (bat_voltage < 3.45) display.println("BATTERY LOW");
  display.display();
}
