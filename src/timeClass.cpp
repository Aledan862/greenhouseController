#include <Arduino.h>

class SysTime {
public:
  uint8_t sec, min, hour, day;
  void tick() {
    sec = (millis() / 1000)%60;
    if ((last_sec == 59) && (sec == 0)) {
      min = (min+1)%60;
    }
    if ((last_min == 59) && (min == 0)) {
      hour = (hour+1)%24;
    }
    if ((last_hour == 23) && (hour == 0)) {
      day = (day+1)%365;
    }
  last_sec = sec;
  last_min = min;
  last_hour = hour;
  };
private:
  uint8_t last_sec, last_min, last_hour;
};
