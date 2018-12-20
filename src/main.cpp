#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define ONE_WIRE_BUS 2
#define THERM_NUM 2  //количество термометров
#define AI_NUM 4     // количество аналогов
#define DO_NUM 4     // дискретные выходы
#define PUBLISH_DELAY 5000

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
DeviceAddress Therm_Address [THERM_NUM] ={
   { 0x28, 0x69, 0x34, 0x77, 0x91, 0x0B, 0x02, 0xE2 },
   { 0x28, 0x85, 0xC7, 0x5B, 0x1E, 0x13, 0x01, 0x79 }
 };
float a [AI_NUM];
float therm [THERM_NUM];

float sp;
String strData = "";
boolean recievedFlag = true;
boolean k [DO_NUM+1];


volatile boolean flag;
uint8_t counter = 1;
long previousMillis;

float getTemperature(DeviceAddress deviceAddress) {

  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    return NAN;
  } else {
    return tempC;
  }
}

float readAnalog(int analogPin, long minimum, long maximum) {
	int rawReading = analogRead(analogPin);
	return (maximum-minimum)*(rawReading / 1023.0) + minimum;
}

void readAIs () {
  // настройки диапазонов входных аналоговых сигналов
  long minmaxSetting [AI_NUM] [2] = {
    {0 , 5},     //канал 0
    {10 , 100}   //канал 1
  };

  for (byte i =0; i < AI_NUM; i++) {
    a [i] = readAnalog(i, minmaxSetting[i][0], minmaxSetting[i][1]);
  }
}

void readThemperatures() {
sensors.requestTemperatures();
  for (byte i =0; i < THERM_NUM; i++) {
    therm [i] = getTemperature(Therm_Address[i]);
  }
}

void readDOs() {
  for (byte i =1; i < DO_NUM+1; i++) {
    k [i] = !digitalRead(7+i-1);
  }
}

void writeDOs() {
  for (byte i =1; i < DO_NUM+1; i++) {
    digitalWrite(7+i-1, !k[i]);
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

void drawscreenTempMoistSoil(uint8_t zoneNum) {

display.clearDisplay();
display.setTextSize(1);
display.setCursor(15,0);
display.print(utf8rus(F("Почва. Зона ")));display.println(zoneNum);
//display.setCursor(0,10);
display.setTextSize(2);
display.setTextColor(WHITE);
display.print(therm[zoneNum-1], 2); display.print("\xB0"); display.println("C"); // Температура с датчика
display.setTextSize(1);
display.print(utf8rus(F("Влажн. "))); display.print("40"); display.print("%"); //влажность
display.display();
display.clearDisplay();
}

void drawscreenHumTempAir() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(20,0);
  display.println(utf8rus(F("Воздух")));
  display.setCursor(0,10);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.print(therm[0], 0); display.print("\xB0"); display.print("C "); // Температура с датчика DTH
  display.print("40"); display.print("%");                                 //влажность
  display.display();
  display.clearDisplay();
}


void drawscreenRelayStatus() {
int x = display.width() / (DO_NUM + 1);
int y = display.height()/2;
int r = (x / 2) - 2;
  for (int16_t i=0; i<DO_NUM; i++) {
    if (k[i]){
      display.fillCircle(r + x*i, y , r, WHITE);
    } else {
      display.drawCircle(r + x*i, y , r, WHITE);
    }
    display.display();
  }
}


void change_screen () {
  if (millis() - previousMillis > 300) {
    //flag =1;
    previousMillis = millis();
    counter = (counter < 4) ? counter+1 : 1;
  }
}

void drawscreen(){
  if (counter < 3){
    drawscreenTempMoistSoil(counter);
  } else {

    switch (counter) {
      case 3:
      drawscreenHumTempAir();
      break;
      case 4:
      drawscreenRelayStatus();
      break;
      }
  }
}

void setup() {
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  for (uint16_t i = 0; i<THERM_NUM; i++){
    sensors.setResolution(Therm_Address [i], 11);
    }
  // setup input pins
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, change_screen, FALLING);
  // setup output pins
  pinMode(7, OUTPUT);
	pinMode(8, OUTPUT);
	pinMode(9, OUTPUT);
	pinMode(10, OUTPUT);

  for (byte i =0; i < 4; i++) {
    k[i+1] = 0;
    a[i] = 0;
  }

  // setup serial communication
  Serial.begin(9600);
  // Clear the buffer

  for (uint8_t i = 0; i<DO_NUM+1; i++){
    Serial.print(k[i]); Serial.print(" ");
  }
  Serial.print("\n");

  Serial.println("Стартуем\n");
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  display.clearDisplay();
  display.cp437(true);
  //drawscreen1();
  Serial.println("все 0\n");
  for (uint8_t i=7; i < 11; i++){
    digitalWrite(i, false);
  }
delay(200);

for (uint8_t i=7; i < 11; i++){
  digitalWrite(i, HIGH);
  delay(100);
  digitalWrite(i, LOW);
  }

}

void loop() {
  readThemperatures();
  readAIs();
  readDOs();


  for (uint8_t i=0; i < DO_NUM+1; i++){
    k[i]=(i == counter) ? 1 : 0;
  }

  writeDOs();

  drawscreen();
  // it's time to send new data?
   /*if (millis() - previousMillis > PUBLISH_DELAY) {
     counter = (counter < 4) ? counter+1 : 1;
     Serial.println(counter);
     for (uint8_t i = 0; i<AI_NUM; i++){
       Serial.print(a[i], 2); Serial.print(" ");
     }
     Serial.print("\n");
     for (uint8_t i = 0; i<THERM_NUM; i++){
       Serial.print(therm[i], 2); Serial.print(" ");
     }
     Serial.print("\n");
     for (uint8_t i = 1; i<DO_NUM+1; i++){
       Serial.print(k[i]); Serial.print(" ");
     }
     Serial.print("       K"); Serial.print(counter);
     Serial.print("\n");
     previousMillis = millis();
   }*/

  // Wait 1 seconds before next cicle
  //delay(PUBLISH_DELAY);
}
