#include <ESP8266WiFi.h>
// #include <WiFiClient.h>
#include <PubSubClient.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ArduinoJson.h>

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme; // I2C


#define SEALEVELPRESSURE_HPA (1013.25)
#define MSG_BUFFER_SIZE  (50)

/* Set these to your desired credentials. */
const char *ssid = "tarix)c_"; //Enter your WIFI ssid
const char *password = "$RFV4rfv"; //Enter your WIFI password
const char* mqtt_server = "192.168.10.201";
const char* MQTT_username = "homeassistant"; 
const char* MQTT_password = "651209"; 
//
int sensorNumber = 1;
String mqttName = "Plant sensor " + String(sensorNumber);
String stateTopic = "home/plants/" + String(sensorNumber) + "/state";

unsigned long lastMsg = 0;
char msg[MSG_BUFFER_SIZE];
int value = 0;
unsigned long delayTime;

//
float humidity;
float temperature;
float pressure;
float altitude;


void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void publisch_values() {
  humidity = bme.readHumidity();
  temperature = bme.readTemperature();
  pressure = bme.readPressure();
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
  if (isnan(humidity)) {
      humidity = 0;
  }   
  if (isnan(temperature)) {
      temperature = 0;
  }
  if (isnan(pressure)) {
      pressure = 0;
  }
  if (isnan(altitude)) {
      altitude = 0;
  }
  // Map moisture sensor values to a percentage value

  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["humidity"] = humidity;
  doc["temperature"]   = temperature;
  doc["moisture"] = humidity; // not implemented now
  doc["pressure"] = pressure; // not implemented now
  doc["altitude"] = altitude; // not implemented now

  size_t n = serializeJson(doc, buffer);

  bool published = client.publish(stateTopic.c_str(), buffer, n);
}

void setup_wifi() {

  Serial.print("Configuring access point...");
  delay(1000);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros()); // what is this ?

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  delay(1000);
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
//    String clientId = "ESP8266Client-";
//    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(mqttName.c_str(), MQTT_username, MQTT_password)) {
      Serial.println("connected to mqtt");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");

      sendMQTTTemperatureDiscoveryMsg();
      sendMQTTHumidityDiscoveryMsg();
      sendMQTTMoistureDiscoveryMsg();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void sendMQTTTemperatureDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/plant_sensor_" + String(sensorNumber) + "/temperature/config";

  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["name"] = "Plant " + String(sensorNumber) + " Temperature";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.temperature|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  client.publish(discoveryTopic.c_str(), buffer, n);
}

void sendMQTTHumidityDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/plant_sensor_" + String(sensorNumber) + "/humidity/config";

  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["name"] = "Plant " + String(sensorNumber) + " Humidity";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "%";
  doc["dev_cla"] = "humidity";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.humidity|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  client.publish(discoveryTopic.c_str(), buffer, n);
}

void sendMQTTMoistureDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/plant_sensor_" + String(sensorNumber) + "/moisture/config";

  DynamicJsonDocument doc(1024);
  char buffer[256];

  doc["name"] = "Plant " + String(sensorNumber) + " Moisture";
  doc["stat_t"]   = stateTopic;
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.moisture|default(0) }}";

  size_t n = serializeJson(doc, buffer);

  client.publish(discoveryTopic.c_str(), buffer, n);
}



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delay(10000);
  Serial.begin(115200);
  Serial.println();
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
//  Serial.print("Configuring access point...");
//  delay(1000);
//  WiFi.begin(ssid, password);
//  while (WiFi.status() != WL_CONNECTED) {
//    delay(500);
//    Serial.print(".");
//  }
//  Serial.println("");
//  Serial.println("WiFi connected");
//  Serial.println("IP address: ");
//  Serial.println(WiFi.localIP());
//  delay(1000);
  

  // BME280 sensor section
  Serial.println(F("BME280 test"));
  unsigned status;
  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;
  printValues();

  Serial.println();
}
void loop() {
  server.handleClient();
  // mqtt
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, MSG_BUFFER_SIZE, "hello world #%ld", value);
    Serial.print("Publish message: ");
    Serial.println(msg);
//    client.publish("outTopic", msg);
    // printValues();
    publisch_values();
  }
  
   // printValues();
   delay(delayTime);
}
