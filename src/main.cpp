#include <Arduino.h>
//#include <OneWire.h>
#include <DallasTemperature.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include "ASOLED.h"
//#include <UIPEthernet.h>
//#include <PubSubClient.h>
#include <DHT.h>
//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 32 // OLED display height, in pixels


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//#define OLED_RESET 4 // Reset pin # (or -1 if sharing Arduino reset pin)
//Adafruit_SSD1306 display(128, 32, &Wire, OLED_RESET);


#define ONE_WIRE_BUS 2
#define AIRDATA 14    //подключение датчика влажности 14(A0)
#define THERM_NUM 2  //количество термометров
#define AI_NUM 5     // количество аналогов
#define DO_START 7  //с какого пина начинается отсчет
#define DO_NUM 3     // дискретные выходы
#define PUBLISH_DELAY 5000
/*/setup ethernet communication-------------------------------------------------
#define CLIENT_ID "GHC2"
#define TOPIC_TEMP  "GHC2/temp"
#define TOPIC_OUT "GHC2/relay"
#define TOPIC_TEMP_SP "GHC2/temp_sp"
IPAddress mqttServer(185,228,232,60);

// MAC-адрес нашего устройства
byte mac[] = { 0x00, 0x2A, 0xF6, 0x12, 0x68, 0xFC };
// ip-адрес устройства
IPAddress ip(192, 168, 0, 115);

EthernetClient ethClient;
PubSubClient clt(ethClient);
*///-----------------------------------------------------------------------------
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
DeviceAddress Therm_Address [THERM_NUM] ={
   { 0x28, 0x76, 0xAB, 0x77, 0x91, 0x11, 0x02, 0x4E  },
   { 0x28, 0x69, 0x34, 0x77, 0x91, 0x0B, 0x02, 0xE2  }
 };
//Air measurement
DHT air;
uint32_t airDelay;

float a [AI_NUM];
float therm [THERM_NUM];

boolean recievedFlag = true;
boolean k [DO_NUM+1];


volatile boolean flag;
uint8_t counter;
long previousMillis;
long NewTime, OldTime;
//------------------------------------------------------------------------------
/*
void callback(char* topic, byte* payload, unsigned int length) {
  String receivedString;
  Serial.print(F("Message arrived ["));
  Serial.print(topic);
  Serial.print(F("] "));

  for (unsigned int i = 0; i < length; i++) {
    receivedString += (char)payload[i];
    }
   if (strcmp(topic,"GHC2/in") == 0) {
    (receivedString == "ON") ? digitalWrite(4, HIGH) : digitalWrite(4, LOW);
    (receivedString == "ON") ? Serial.println("on") : Serial.println("off");
    }
}*/


/*
void reconnect() {
//  Serial.println(F("start reconnect function..."));
  // Loop until we're reconnected
//  while (!clt.connected()) {
//    Serial.println(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (clt.connect(CLIENT_ID)) {
//      Serial.println(F("Connected"));
      // ... and subscribe to topics
    //  clt.subscribe("inTopic");
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(clt.state());
//      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      //delay(5000);
    //}
  }

}


long lastRecAtt = 0;

boolean reconnect() {
  Serial.println(CLIENT_ID);
  Serial.print(clt.connect("arduinoClient"));
  if (clt.connect("arduinoClient")) {
    // Once connected, publish an announcement...
    clt.publish("Status","reconnected");
    // ... and resubscribe
    clt.subscribe("in");
  }
  Serial.print("#########################point2");
  return clt.connected();
}
*/
//-----------------------------------------------------------------------------

long sys_sec, sys_min, sys_hour;
boolean firstexecution = true;
int8_t last_sys_sec;
void systemtime() {
  sys_sec = (millis() / 1000)%60;
  if ((last_sys_sec == 59) && (sys_sec == 0)) {
    sys_min = (sys_min+1)%60;
  }
last_sys_sec = sys_sec;
firstexecution = false;
}


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
    {0 , 100}   //канал 1
  };
  for (byte i =1; i < AI_NUM; i++) {
    if (i<4) {
      a [i] = readAnalog(i, minmaxSetting[i][0], minmaxSetting[i][1]);
    } else {
      a [i] = readAnalog(i+2, minmaxSetting[i][0], minmaxSetting[i][1]);
    }
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
    k [i] = !digitalRead(DO_START+i-1);
  }
}

void writeDOs() {
  for (byte i =1; i < DO_NUM+1; i++) {
    digitalWrite(DO_START+i-1, !k[i]);
  }

}

void discretRegul(float pv, float sp, float deadband, int outport ) {
  if ((pv > sp + deadband) and !digitalRead(outport)) {
    digitalWrite(outport, HIGH);
  } else if  ((pv < sp - deadband) and digitalRead(outport)) {
    digitalWrite (outport, LOW);
  }
}


long delayOn, delayOff;
void timer () {
  if (sys_min > delayOff) {
      k[1] = 1;} else {
        k[1]=0;
      }
}


void drawscreenTempMoistSoil(uint8_t zoneNum) {
/*
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
*/
LD.printString_12x16(F("Почва Зона "),0,0); LD.printNumber((long)zoneNum,13,2);
LD.printNumber(therm[zoneNum-1],1,3,4); LD.printString_12x16("\xC2\xB0",60,4); LD.printString_12x16(F("C"));
LD.printNumber(a[zoneNum],1,4,6);LD.printString_12x16(F("%"),60,6);
//LD.clearDisplay();
}


void drawscreenHumTempAir() {
  float humidity = air.getHumidity();
  float temperature = air.getTemperature();

  LD.printString_12x16(F("Воздух"),0,0);
  LD.printNumber(temperature,1,4,3); LD.printString_12x16("\xC2\xB0",60,3); LD.printString_12x16(F("C"));
  LD.printNumber(humidity,1,4,5);LD.printString_12x16(F("%"),60,5);
  //LD.clearDisplay();
}

/*
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
*/

void change_screen() {
  if (millis() - previousMillis > 300) {
    //flag =1;
    previousMillis = millis();
    counter = ++counter % 4;
    counter = (counter == 0) ? 1 : counter;
    OldTime = 0;
  }
}

void drawscreen() {
/*  if (counter == 0){
    LD.clearDisplay();
  } else{

    if (counter < 3){
      drawscreenTempMoistSoil(counter);
    } else {
        switch (counter) {
          case 3:
          drawscreenHumTempAir();
          break;
          case 4:
          //drawscreenRelayStatus();
          break;
      }
    }
  }*/
switch (counter) {
  case 0:
    LD.clearDisplay();
    break;
  case 1:
    drawscreenHumTempAir();
    break;
  case 2:
    drawscreenTempMoistSoil(1);
    break;
  case 3:
    drawscreenTempMoistSoil(2);
    break;
}
}

void setup(){
  sensors.begin();
  // set the resolution to 10 bit (good enough?)
  for (uint16_t i = 0; i<THERM_NUM; i++){
    sensors.setResolution(Therm_Address [i], 11);
    }
  //Setup air sensor
  air.setup(AIRDATA);
  //airDelay = air.getMinimumSamplingPeriod();

  // setup input pins
  pinMode(3, INPUT_PULLUP);// button
  attachInterrupt(1, change_screen, FALLING);

  // setup output pins
  for (uint8_t i = DO_START; i < DO_START+DO_NUM; i++){
    pinMode(i, OUTPUT);
    digitalWrite(i, 1);
  }
  //setup memory table to zero
  for (byte i =0; i < 4; i++) {
    k[i+1] = 0;
    a[i] = 0;
  }
  //setup timeouts
  delayOn = 1 * 60000  ;  //1 минута
  delayOff = 40 ;  //2 минута

  // setup serial communication
  Serial.begin(9600);
  Serial.println(F("Стартуем\n"));
  LD.init();  //initialze OLED display
  LD.clearDisplay();
/*----------------------------------------------------------------------
  clt.setServer(mqttServer, 1883);
  //clt.setCallback(callback);
  //Ethernet.begin(mac, ip);
  Serial.println(F("Ethernet configured"));
  Serial.print(F("IP address: "));
  Serial.println(Ethernet.localIP());
  Serial.println(Ethernet.subnetMask());
  Serial.println();
  Serial.println(F("Ready to send data"));
}

void sendData() {
  char msgBuffer[20];
  if (clt.connect(CLIENT_ID)) {
    clt.publish(TOPIC_TEMP, dtostrf(therm[0], 6, 2, msgBuffer));
    clt.publish(TOPIC_TEMP_SP, dtostrf(therm[1], 6, 2, msgBuffer));
  //  clt.publish(TOPIC_OUT, (k[0] == HIGH) ? "Остываем" : "Греем");
    Serial.println(dtostrf(therm[0], 6, 2, msgBuffer));
  }

}
//------------------------------------------------------------------------*/
}

void loop() {
  systemtime();
  readThemperatures();
  readAIs();
  readDOs();
/*
  for (uint8_t i=0; i < DO_NUM+1; i++){
    k[i]=(i == counter) ? 1 : 0;
  }
*/
  timer();
  writeDOs();

  NewTime = millis();
  if (NewTime - OldTime > PUBLISH_DELAY) {
      LD.clearDisplay();
      drawscreen();
      /*sendData();
      clt.publish("outTopic","test");
      Serial.print("outTopic:");Serial.println("test");*/
      OldTime = millis();
  }
if (NewTime - previousMillis > 5*PUBLISH_DELAY ) {
  counter = 0;
}
Serial.println(counter);
}
