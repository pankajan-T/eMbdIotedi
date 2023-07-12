
//Libraries used
#include <PubSubClient.h>
#include <WiFi.h>
//#include <WiFiManager.h>
#include "DHTesp.h"

#include <NTPClient.h>
#include <WiFiUdp.h>


#include <LiquidCrystal_I2C.h>

#include <ESP32Servo.h>


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
char LDRAr[6];


//Wifi NTP client declaration
WiFiUDP udp;
NTPClient timeClient(udp);

// Buzzer decalarations
const int Buzzer_PIN = 2;
unsigned int frequency = 256;
unsigned long duration = 10; 


//LCD I2C declaration

LiquidCrystal_I2C lcd (0x27, 16,2);

///////LDR declaration

const int LDRPIN = 34;
float LDRValue;


#define SERVO_PIN 26 // ESP32 pin GIOP26 connected to servo motor
Servo servoMotor;
float pos;
float offset = 30.0;
float cntFact = 0.75;

//////////////////////Alarm declarations

///1st slot

int hour[3]={17,0,0};
int minuite[3]={26,0,0};
int slot[3]={1,0,0};



//////////////////////


using namespace std;



void setup() {
  Serial.begin(115200);
  setupWiFi();
  setupMqtt();


  timeClient.begin();

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

  pinMode(Buzzer_PIN, OUTPUT);


  lcd.init();
  lcd. backlight ();

  servoMotor.attach(SERVO_PIN);
  

 
}

void loop() {
  if(!mqttClient.connected()){


    connectToBroker();

  }

  timeClient.update();
  //Serial.println(timeClient.getFormattedTime());




  mqttClient.loop();


  //LDR read
  LDRValue = analogRead(LDRPIN)/4095.0;
  updateLDR();


  //publishing for temperature sensor value to mqtt
  updateTemperatureandHumidity();
  mqttClient.publish("ENTC-TEMP",tempAr);
  mqttClient.publish("ENTC-HUM",humAr);
  
  //publish LDR ~light intensity
  mqttClient.publish("LDR",LDRAr);

  delay(500);

  lcd. setCursor (0, 0);
   // We write the number of seconds elapsed 
  lcd. print (timeClient.getFormattedTime());
  lcd.setCursor(0,1);
  lcd.print(tempAr);
  lcd.print("`C  ");

  lcd.print(humAr);
  

  Alarmcheck();
  

  pos = offset+(180-offset)*LDRValue*cntFact;
  servoMotor.write(pos);
  

  //Serial.println(pos);
  
  


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
      mqttClient.subscribe("Offseter");
      mqttClient.subscribe("cntFactor");
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

void updateLDR(){

String(LDRValue,2).toCharArray(LDRAr,6);

}




/// recieved message from the node red to esp32

void recieveCallback(String topic, byte* payload, unsigned int length){

  Serial.print("message arrived[");
  Serial.print(topic);
  Serial.print("]");


  char payloadCharAr[length];

  for(int i=0; i<length;i++){
  payloadCharAr[i] = (char)payload[i];
  Serial.print((char)payload[i]);
  }
  //Serial.println(topic == "BuzzerFreq");

  if(topic == "BuzzerFreq"){
    frequency =atoi(payloadCharAr);
    //Serial.println("test");
  }
  else if(topic == "BuzzerDur"){
    duration = atoi(payloadCharAr);

  }

  else if(topic == "Offseter"){
    offset = atoi(payloadCharAr);
    //Serial.println("test");
  }
  else if(topic == "cntFactor"){
    cntFact = atoi(payloadCharAr);
  }

  


}


void Alarmcheck(){


  int currentHour = timeClient.getHours();
  int currentMin = timeClient.getMinutes();
 

  for(int i =0; i<3;i++){
    
    if(slot[i]==1){
      if(hour[i]==currentHour && minuite[i]==currentMin){
      Serial.println("Alarm");
      tone(Buzzer_PIN, frequency, duration);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Time For Pill!");
      break;
      }
    }


  }


}