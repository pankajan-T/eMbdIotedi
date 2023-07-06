#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiManager.h>

WiFiClient espClient;
PubSubClient mqttClient(espClient);


void setup() {
  Serial.begin(115200);
  setupWiFi();
  setupMqtt();


 
}

void loop() {
  if(!mqttClient.connected()){


    connectToBroker();

  }

  mqttClient.loop();
  //publishing for temperature sensor value to mqtt
  mqttClient.publish("ENTC-TEMP","25.24");
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

