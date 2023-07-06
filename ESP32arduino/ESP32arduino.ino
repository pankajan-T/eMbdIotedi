#include <PubSubClient.h>
#include <WiFi.h>

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
  


}


void connectToBroker(){
  while(!mqttClient.connected()){
    Serial.print("Attempting MQTT connection....");
    if(mqttClient.connect("ESP32-4635")){

      Serial.println("connected");

    }
    else{
      Serial.println("failed");
      Serial.print(mqttClient.state());
      delay(5000);
    }

  }


}

void setupWiFi(){

}

void setupMqtt(){



}

