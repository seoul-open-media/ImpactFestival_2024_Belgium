void getHWSerial() {
  if (HWSERIAL.available() > 22) {

//    Serial.println("HWSERIAL");
    // check my bow state
    // my_bow_state = digitalRead(mercurySwitchPin);

    //  Serial.println("Got Final Result");
    // digitalWrite(13, HIGH);
    byte first_byte = HWSERIAL.read();
    byte second_byte = HWSERIAL.read();
    // Serial.print("first_byte ="); Serial.println(first_byte);
    // Serial.print("second_byte ="); Serial.println(second_byte);
    if (first_byte == 255 && second_byte == FINAL_RESULT) {

      // read the raging rusult
      for (int i = 0; i < 20; i++) {
        distance_result[i] = HWSERIAL.read();
        //if (distance_result[i] == 0)MY_ADDRESS = (i + 1);
//        Serial.print(distance_result[i]);
//        Serial.print(", ");
      }
//      Serial.println();
      // read elapsed_time
      elapsed_time = HWSERIAL.read(); // s_data[22]

//      Serial.print("eleapsed time: ");
//      Serial.println(elapsed_time);
      /*
        for (int i = 0; i < 80; i++) {
        _info[i] = HWSERIAL.read();
        //if (distance_result[i] == 0)MY_ADDRESS = (i + 1);
        Serial.print(_info[i]);
        Serial.print(", ");
        }

        Serial.println();
      */
      checksum_received = HWSERIAL.read(); // s_data[103]

      // Serial.print("checksum ");
      // Serial.println(checksum_received);

//      displayData05();

      isGetPosition = true;

    }
    HWSERIAL.clear();
  }
}
