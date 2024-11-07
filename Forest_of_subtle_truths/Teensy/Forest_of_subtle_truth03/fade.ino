void fadeZone1() {
  destination_gain[0] = 1;
  destination_gain[1] = 0;
  destination_gain[2] = 0;
  destination_gain[3] = 0;
}

void fadeZone2() {
  destination_gain[0] = 0;
  destination_gain[1] = 1;
  destination_gain[2] = 0;
  destination_gain[3] = 0;
}

void fadeZone3() {
  destination_gain[0] = 0;
  destination_gain[1] = 0;
  destination_gain[2] = 1;
  destination_gain[3] = 0;
}

void fadeZone4() {
  destination_gain[0] = 0;
  destination_gain[1] = 0;
  destination_gain[2] = 0;
  destination_gain[3] = 1;
}
