#include <Adafruit_SleepyDog.h>
#include <SoftwareSerial.h>
#include "Adafruit_FONA.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_FONA.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP280.h>

/* Pins */
// Default pins for Feather 32u4 FONA
#define FONA_RX  9
#define FONA_TX  8
#define FONA_RST 4
#define ONE_WIRE_BUS 5

/* APN details */
#define FONA_APN       "everywhere"
#define FONA_USERNAME  "eesecure"
#define FONA_PASSWORD  "secure"

/* Adafruit.io Setup */
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    ""
#define AIO_KEY         ""

/* Global State */
SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);

Adafruit_MQTT_FONA mqtt(&fona, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
boolean FONAconnect(const __FlashStringHelper *apn, const __FlashStringHelper *username, const __FlashStringHelper *password);

int dtrPin = 5;
uint8_t tx_failures = 0;
#define MAX_TX_FAILURES 3

/* Feeds */
Adafruit_MQTT_Publish DS18B20_sensor_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DS18B20_sensor");
Adafruit_MQTT_Publish BMP280_sensor_feed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/BMP280_sensor");

/* DS18B20 */
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20_sensor(&oneWire);

/* BMP280 */
Adafruit_BMP280 bmp;
Adafruit_Sensor *bmpPressure = bmp.getPressureSensor();


void setup() {
  //while (!Serial); // removed for standalone state

  Watchdog.enable(8000);

  pinMode(dtrPin, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("Adafruit FONA MQTTp demo"));
  waitFiveSeconds();

  while (! FONAconnect(F(FONA_APN), F(FONA_USERNAME), F(FONA_PASSWORD))) {
    Serial.println("Retrying FONA");
  }
  Serial.println(F("Connected to Cellular!"));

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

   /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500); 

  waitFiveSeconds();
}

void loop() {
  Watchdog.reset();
  MQTT_connect();
  Watchdog.reset();
  
  float t = getTempReading();

  if (isnan(t)) {
    Serial.println(F("Failed to read from DS18B20 sensor!"));
    Watchdog.reset();
    return;
  }

  Serial.print(F("\nSending temp val"));
  Serial.print(t);

  if (!DS18B20_sensor_feed.publish(t)) {
    Serial.println(F("Failed"));
    tx_failures++;
  } else {
    Serial.println(F("OK!"));
    tx_failures = 0;
  }

  sensors_event_t pressureEvent;
  bmpPressure->getEvent(&pressureEvent);
  
  if (!BMP280_sensor_feed.publish(pressureEvent.pressure)) {
    Serial.println(F("Failed"));
    tx_failures++;
  } else {
    Serial.println(F("OK!"));
    tx_failures = 0;
  }

  if (tx_failures >= MAX_TX_FAILURES){
    // put code here to handle tx failures
  }

  sleepFONAModule();
  waitFiveSeconds();
  
  int number_of_sleeper_loops = 10; // 30 1min, 300 10mins
  for (int i = 0; i < number_of_sleeper_loops; i++) {
      Watchdog.sleep(2000);
  }

  waitFiveSeconds();
  wakeupFONAModule();
  waitFiveSeconds();
}

void waitFiveSeconds() {
  Watchdog.reset();
  delay(5000);
  Watchdog.reset();
}

void sleepFONAModule() {
  Watchdog.reset();
  digitalWrite(dtrPin, HIGH);
  fonaSS.begin(4800);
  fonaSS.println(F("AT+CFUN=0")); 
  Watchdog.reset();
}

void wakeupFONAModule() {
  Watchdog.reset();
  digitalWrite(dtrPin, LOW);
  delay(60);
  fonaSS.begin(4800);
  fonaSS.println(F("AT+CFUN=1"));
}

float getTempReading() {
  int number_of_readings = 3;
  float temp_array[number_of_readings];

  for (int i = 0; i < number_of_readings; i++) {
    DS18B20_sensor.requestTemperatures();
    float t = DS18B20_sensor.getTempCByIndex(0);
    temp_array[i] = t;
    waitFiveSeconds();
  }

  float sum_t = 0;
  for (int i = 0; i< number_of_readings; i++)
  {
    sum_t += temp_array[i];
  }

  float avg_t = sum_t / number_of_readings;
  return avg_t;
}

void MQTT_connect() {
  int8_t ret;

  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Retrying MQTT connection in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
  }
  
  Serial.println("MQTT Connected!");
}
