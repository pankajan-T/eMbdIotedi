
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


char daysAr[6];


//Wifi NTP client declaration
WiFiUDP udp;
NTPClient timeClient(udp, "lk.pool.ntp.org", 530*6*6,30000);

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

int mainswitch =1;

int hour[3]={17,0,0};
int minuite[3]={26,0,0};
int slot[3]={1,0,0};

bool AlarmOnOff = 1;

int alarming=0;
int alarminuite =00;
int buzzerType =0;

int days = 5;

//////////////////////


using namespace std;



void setup() {
  Serial.begin(115200);
  setupWiFi();
  setupMqtt();


  timeClient.begin();
  timeClient.setTimeOffset(19080);

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

  //main swithc

  if(mainswitch==1){


  //publishing for temperature sensor value to mqtt
  updateTemperatureandHumidity();
  mqttClient.publish("ENTC-TEMP",tempAr);
  mqttClient.publish("ENTC-HUM",humAr);
  
  //publish LDR ~light intensity
  mqttClient.publish("LDR",LDRAr);


  lcd. setCursor (0, 0);
   // We write the number of seconds elapsed 
  lcd. print (timeClient.getFormattedTime());
  lcd.setCursor(0,1);
  lcd.print(tempAr);
  lcd.print("`C  ");

  lcd.print(humAr);
  
  if(AlarmOnOff){
  Alarmcheck();
  }

  pos = offset+(180-offset)*LDRValue*cntFact;
  servoMotor.write(pos);
  

  //Serial.println(pos);
  
  //tone(Buzzer_PIN, frequency, duration);

  }
  else{
   lcd. setCursor (0, 0);
   lcd.print("Main Switch Off");
  }
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

      mqttClient.subscribe("scheduleOnOff");
      mqttClient.subscribe("days");
      mqttClient.subscribe("slot1switch");
      mqttClient.subscribe("slot2switch");
      mqttClient.subscribe("slot1time");
      mqttClient.subscribe("slot2time");
      mqttClient.subscribe("slot3switch");
      mqttClient.subscribe("slot3time");
      mqttClient.subscribe("buzzerType");
      mqttClient.subscribe("Mainswitch");


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

  int temphour;

  Serial.print("message arrived[");
  Serial.print(topic);
  Serial.print("]  - ");


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

  else if(topic == "days"){
    days = atoi(payloadCharAr);
  }

else if(topic == "slot1switch"){
  if((char)payload[0]=='1'){
    slot[0] = 1;
  }

  else if((char)payload[0]=='0'){
    slot[0] = 0;
  }
  
  
  }

else if(topic == "slot1time"){
  //Serial.println(atoi(payloadCharAr));
    temphour = atoi(payloadCharAr)/100;
    hour[0] = temphour;

    minuite[0] = atoi(payloadCharAr) -temphour*100;

  }

else if(topic == "slot2switch"){

 
  if((char)payload[0]=='1'){
   
    slot[1] = 1;
  }

  else if((char)payload[0]=='0'){
    slot[1] = 0;
  }
  
  }

else if(topic == "slot2time"){
  temphour = atoi(payloadCharAr)/100;
    hour[1] = temphour;

    minuite[1] = atoi(payloadCharAr) -temphour*100;

    
  }

else if(topic == "slot3switch"){
  if((char)payload[0]=='1'){
    slot[2] = 1;
  }

  else if((char)payload[0]=='0'){
    slot[2] = 0;
  }
  
  }

else if(topic == "slot3time"){
  temphour = atoi(payloadCharAr)/100;
    hour[2] = temphour;

    minuite[2] = atoi(payloadCharAr) -temphour*100;

    
  }

else if(topic == "scheduleOnOff"){
    

if((char)payload[0]=='1'){
    AlarmOnOff = 1;
  }

  else if((char)payload[0]=='0'){
    AlarmOnOff =0;
  }
  
  }

  else if(topic == "buzzerType"){
    

if((char)payload[0]=='1'){
    buzzerType =1;
  }

  else if((char)payload[0]=='0'){
    buzzerType =0;
  }
  
  }


  else if(topic == "Mainswitch"){
  if((char)payload[0]=='1'){
    mainswitch = 1;
  }

  else if((char)payload[0]=='0'){
    mainswitch = 0;
  }
  
  
  }
    
  }


  





void Alarmcheck(){


  int currentHour = timeClient.getHours();
  int currentMin = timeClient.getMinutes();
 
  if( days > 0){
  for(int i =0; i<3;i++){
    
    if(slot[i]==1){
      if(hour[i]==currentHour && minuite[i]==currentMin){
      Serial.println("Alarm");
      if(buzzerType ==0){
        //Serial.println("Test");
      tone(Buzzer_PIN, frequency, duration);
      delay(500);
      }
      else{
        digitalWrite(Buzzer_PIN,HIGH);
        tone(Buzzer_PIN, 1, duration);
        
      }
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Time For Pill!");
      alarminuite =currentMin;

      alarming=1;

      break;
      }
    }


  }
  }

  
  
  
/////After sometime stop the alarm
  if(currentMin==alarminuite+duration && alarming ==1){

    lcd.clear();
    digitalWrite(Buzzer_PIN,LOW);
    alarming=0;
  }



////// days update

if(currentHour ==0 && currentMin==0 && days >=0){
  days = days-1;
  String(days,1).toCharArray(daysAr,6);

  mqttClient.publish("daysIn",daysAr);
  




}

}