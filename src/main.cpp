#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ONE_WIRE_BUS 2
#define PUBLISH_DELAY 3000
#define SP_MAX 90
#define SP_MIN 10

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

DeviceAddress Thermometer1 = { 0x28, 0x69, 0x34, 0x77, 0x91, 0x0B, 0x02, 0xE2};
int Relay_pin = 4;
int sp_temperature_Pin = A0;
float t;
long previousMillis;
float sp;
String strData = "";
boolean recievedFlag = true;

// 'heat', 18x18px
const unsigned char heat [] PROGMEM = {
	0xfd, 0xb7, 0xc0, 0xfb, 0x6f, 0xc0, 0xfd, 0xb7, 0xc0, 0xfb, 0x6f, 0xc0, 0xff, 0xff, 0x80, 0xe4,
	0x93, 0x80, 0xdb, 0x6d, 0x80, 0xdb, 0x6d, 0x80, 0xdb, 0x6d, 0x80, 0x1b, 0x6c, 0x00, 0xdb, 0x6d,
	0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0,
	0xdb, 0x6d, 0xc0, 0xe4, 0x93, 0xc0
};

// 'cool', 18x18px
const unsigned char cool [] PROGMEM = {
	0xff, 0xff, 0xc0, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xc0, 0xff, 0xff, 0xc0, 0xff, 0xff, 0x80, 0xe4,
	0x93, 0x80, 0xdb, 0x6d, 0x80, 0xdb, 0x6d, 0x80, 0xdb, 0x6d, 0x80, 0x1b, 0x6c, 0x00, 0xdb, 0x6d,
	0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0, 0xdb, 0x6d, 0xc0,
	0xdb, 0x6d, 0xc0, 0xe4, 0x93, 0xc0
};


void currentStatus() {
  Serial.print(F("Temperature: "));
  Serial.print(t);
  Serial.print(F(" Уставка: "));
  Serial.print(sp);
  Serial.print(F(" Состояние: "));
  Serial.println((digitalRead(Relay_pin) == HIGH) ? "Остываем" : "Греем");
}

float readSp(int analogPin) {
  int rawReading = analogRead(analogPin);
  return (SP_MAX-SP_MIN)*(rawReading / 1023.0) + SP_MIN;
}

float getTemperature(DeviceAddress deviceAddress) {

  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    return NAN;
  } else {
    return tempC;
  }
}
void discretRegul(float pv, float sp, float deadband, int outport ) {
  if ((pv > sp + deadband) and !digitalRead(outport)) {
    digitalWrite(outport, HIGH);
  } else if  ((pv < sp - deadband) and digitalRead(outport)) {
    digitalWrite (outport, LOW);
  }
}

String utf8rus(String source)
{
  int i,k;
  String target;
  unsigned char n;
  char m[2] = { '0', '\0' };

  k = source.length(); i = 0;

  while (i < k) {
    n = source[i]; i++;

    if (n >= 0xC0) {
      switch (n) {
        case 0xD0: {
          n = source[i]; i++;
          if (n == 0x81) { n = 0xA8; break; }
          if (n >= 0x90 && n <= 0xBF) n = n + 0x30;
          break;
        }
        case 0xD1: {
          n = source[i]; i++;
          if (n == 0x91) { n = 0xB8; break; }
          if (n >= 0x80 && n <= 0x8F) n = n + 0x70;
          break;
        }
      }
    }
    m[0] = n; target = target + String(m);
  }
return target;
}

void drawscreen1() {
display.display();
display.clearDisplay();
display.setTextSize(1);
display.fillRect(0, 0, 20, 20, WHITE);
if (digitalRead(Relay_pin) == HIGH) {
  display.drawBitmap(1, 1, cool, 18, 18, BLACK);
} else {
  display.drawBitmap(1, 1, heat, 18, 18, BLACK);
};

//display.println(utf8rus(F("ТЕМПЕРАТУРА")));
display.setCursor(22,0);
display.setTextSize(2);
display.setTextColor(WHITE);
display.print(t, 2); display.print("\xB0C");
display.setCursor(32, 15);
display.setTextSize(1);
display.print(sp, 1); display.print("\xB0C");
//display.setTextSize(1);
//display.fillCircle(display.width()-8, 7, 4, (digitalRead(Relay_pin) == HIGH) ? WHITE : BLACK);


//(digitalRead(Relay_pin) == HIGH) ? heat : cool)
//display.println(utf8rus(F("АБВГДЕЖЗИЙКЛМНОП")));
//display.fillCircle(display.width()-12, 3, 4, WHITE)
}

void testdrawstyles(void) {
  for (int16_t i=0; i<display.width(); i+=4) {
    display.drawLine(0, 0, i, display.height()-1, WHITE);
    display.display();
  }
  delay(2000);
}


void setup() {
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(Thermometer1, 10);
  // setup output pins
  pinMode(Relay_pin, OUTPUT);
	for (int8_t i=7; i<11; i+=1){
		pinMode(i, OUTPUT);
	};

  // setup serial communication
  Serial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  // Clear the buffer
  display.clearDisplay();
  display.cp437(true);
  testdrawstyles();
}

void loop() {
  sp = readSp(sp_temperature_Pin);
  sensors.requestTemperatures();
  t = getTemperature(Thermometer1);
  discretRegul(t, sp , 2.0, Relay_pin);
	for (int8_t i=7; i<11; i+=1){
		digitalWrite(i, (digitalRead(i) == HIGH) ? LOW : HIGH);
		delay(200);
	};
  // it's time to send new data?
  if (millis() - previousMillis > PUBLISH_DELAY) {
    currentStatus();
    previousMillis = millis();
  }
  drawscreen1();
  // Wait 1 seconds before next cicle
  delay(1000);

}
