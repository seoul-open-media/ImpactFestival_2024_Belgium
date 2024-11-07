bool pointInPolygon(float x, float y, uint8_t znum)
{
  int i, j = polySides[znum] - 1;
  bool oddNodes = false;

  for ( i = 0; i < polySides[znum]; i++ )
  {
    if ( (zonePolygon[znum][i]._y < y && zonePolygon[znum][j]._y >= y || zonePolygon[znum][j]._y < y && zonePolygon[znum][i]._y >= y) &&  (zonePolygon[znum][i]._x <= x || zonePolygon[znum][j]._x <= x) )
    {
      oddNodes ^= ( zonePolygon[znum][i]._x + (y - zonePolygon[znum][i]._y) / (zonePolygon[znum][j]._y - zonePolygon[znum][i]._y) * (zonePolygon[znum][j]._x - zonePolygon[znum][i]._x) < x );
    }

    j = i;
  }

  return oddNodes;
}
