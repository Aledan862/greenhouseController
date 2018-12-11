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

DeviceAddress Thermometer1 = { 0x28, 0x85, 0xC7, 0x5B, 0x1E, 0x13, 0x01, 0x79 }; //28 85 C7 5B 1E 13 1 79
int Relay_pin = 5;
int sp_temperature_Pin = A0;
float t;
long previousMillis;
float sp;
String strData = "";
boolean recievedFlag = true;

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


void setup() {
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  sensors.setResolution(Thermometer1, 10);
  // setup output pins
  pinMode(Relay_pin, OUTPUT);
  // setup serial communication
  Serial.begin(9600);

}

void loop() {
  sp = readSp(sp_temperature_Pin);
  sensors.requestTemperatures();
  t = getTemperature(Thermometer1);
  discretRegul(t, sp , 2.0, Relay_pin);

  // it's time to send new data?
  if (millis() - previousMillis > PUBLISH_DELAY) {
    currentStatus();
    previousMillis = millis();
  }

  // Wait 1 seconds before next cicle
  delay(1000);

}
