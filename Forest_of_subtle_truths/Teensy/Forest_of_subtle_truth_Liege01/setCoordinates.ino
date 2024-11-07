void setCoordinates(){
  for (int i = 0; i < TOTAL_NUM_ZONE; i++) {
    for (int j = 0; j < GROUP_NUM_POINT; j++) {
      zonePolygon[i][j]._x = zonePointX[i][j]; 
      zonePolygon[i][j]._y = zonePointY[i][j];
    }
  }
}
