
//Libraries used
#include <PubSubClient.h>
#include <WiFi.h>
//#include <WiFiManager.h>
#include "DHTesp.h"

//////////////////////

//Special declarations
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const int DHT_PIN = 15;
DHTesp dhtSensor;
char tempAr[6];



//////////////////////

void setup() {
  Serial.begin(115200);
  setupWiFi();
  setupMqtt();

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

 
}

void loop() {
  if(!mqttClient.connected()){


    connectToBroker();

  }

  mqttClient.loop();
  //publishing for temperature sensor value to mqtt
  updateTemperature();
  mqttClient.publish("ENTC-TEMP",tempAr);
  delay(500);
  


}


void connectToBroker(){

//************Making sure Mqtt is connected
  while(!mqttClient.connected()){
    Serial.print("\n Attempting MQTT connection....");
    if(mqttClient.connect("ESP32-4635")){

      Serial.println("connected");

    }
    else{
      Serial.println("failed");
      Serial.print(mqttClient.state());
      delay(5000);
    }

  }
//*********************************************


}

void setupWiFi(){

WiFi.begin("Wokwi-GUEST","");
while(WiFi.status() != WL_CONNECTED){
delay(500);
Serial.print(".");


}
Serial.println("Connected...!!!");
Serial.println(WiFi.localIP());


}

void setupMqtt(){

  mqttClient.setServer("test.mosquitto.org",1883);


}


void updateTemperature(){

TempAndHumidity data = dhtSensor.getTempAndHumidity();
String(data.temperature,2).toCharArray(tempAr,6);


}
