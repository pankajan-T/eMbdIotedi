
//Libraries used
#include <PubSubClient.h>
#include <WiFi.h>
//#include <WiFiManager.h>
#include "DHTesp.h"

#include <NTPClient.h>
#include <WiFiUdp.h>


//#include <iostream>
//#include<string>

//////////////////////

//Special declarations

//Wifi client delcarations
WiFiClient espClient;
PubSubClient mqttClient(espClient);


// DHT humid&Temp sensor decalarations
const int DHT_PIN = 15;

DHTesp dhtSensor;
char tempAr[6];
char humAr[6];


//Wifi NTP client declaration
WiFiUDP udp;
NTPClient timeClient(udp);

// Buzzer decalarations
const int Buzzer_PIN = 2;
unsigned int frequency = 256;
unsigned long duration = 10; 

//////////////////////


using namespace std;

void setup() {
  Serial.begin(115200);
  setupWiFi();
  setupMqtt();


  timeClient.begin();

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  pinMode(Buzzer_PIN, OUTPUT);    

 
}

void loop() {
  if(!mqttClient.connected()){


    connectToBroker();

  }

  timeClient.update();
  //Serial.println(timeClient.getFormattedTime());




  mqttClient.loop();


  //publishing for temperature sensor value to mqtt
  updateTemperatureandHumidity();
  mqttClient.publish("ENTC-TEMP",tempAr);
  mqttClient.publish("ENTC-HUM",humAr);

  delay(500);



  tone(Buzzer_PIN, frequency, duration);


}


void connectToBroker(){

//************Making sure Mqtt is connected
  while(!mqttClient.connected()){
    Serial.print("\n Attempting MQTT connection....");
    if(mqttClient.connect("ESP32-4635")){

      Serial.println("connected");
      mqttClient.subscribe("ENTC-ON-OFF");
      mqttClient.subscribe("BuzzerFreq");
      mqttClient.subscribe("BuzzerDur");

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
  mqttClient.setCallback(recieveCallback);

}


void updateTemperatureandHumidity(){

TempAndHumidity data = dhtSensor.getTempAndHumidity();
String(data.temperature,2).toCharArray(tempAr,6);
String(data.humidity,2).toCharArray(humAr,6);


}






/// recieved message from the node red to esp32

void recieveCallback(char* topic, byte* payload, unsigned int length){

  Serial.print("message arrived[");
  Serial.print(topic);
  Serial.print("]");


  char payloadCharAr[length];

  for(int i=0; i<length;i++){
  payloadCharAr[i] = (char)payload[i];
  Serial.print((char)payload[i]);

  if(topic == "BuzzerFreq"){
    frequency =atoi(payloadCharAr);

  }
  else if(topic == "BuzzerDur"){
    duration = atoi(payloadCharAr);

  }

  }


}
