#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>


char* ssid = "North";
char* password = "0987654321";
char* mqtt_server = "broker.netpie.io"; //*
int mqtt_port = 1883;  //*
char* mqtt_Client = "aa13e59d-8f6c-4cb8-9edc-1ff015c40d56";
char* mqtt_username = "kxV4TpJV8eah5ZFv81JHYktXiaDh1HU9";
char* mqtt_password = "OJROy825duulEtIHLz7D_tNq7YeqmI1P";

//* = no need to change
long lastMsg = 0;
char msg[100];
char msg2[16];
char msg3[16];
String objDetected = "false";
String cap = "-1";
WiFiClient espClient;
PubSubClient client(espClient);

SoftwareSerial s_serial(D7,D8);

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting NETPIE2020 connection…");
    if (client.connect(mqtt_Client, mqtt_username, mqtt_password)) {
      Serial.println("NETPIE2020 connected");
      client.subscribe("@msg/obj");  //<------------------------
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println("try again in 3 seconds");
      delay(3000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length){   //  <<-------  edit: 18/5/21  activated when a message is arrived
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message; char state;
  for(int i=0;i < length;i++){
    message = message + (char)payload[i];
  }
  Serial.println(message);

  if(String(topic) == "@msg/obj"){
    objDetected = message;
    if(message == "true"){
      state = 65;
      client.publish("@shadow/data/update", "{\"data\": {\"obj\": true}}");
    }
    else if(message == "false"){
      state = 66;
      client.publish("@shadow/data/update", "{\"data\": {\"obj\": false}}");
    }
    Serial.print(state);  //send status to STM32
    s_serial.print(state);
  }
}


void setup() {
  Serial.begin(115200); //baud rate
  s_serial.begin(115200);  //  <<-------  edit: 18/5/21
  Serial.println("Starting...");
  if (WiFi.begin(ssid, password)) { //<------------------
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.print(".");
    }
  }
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);     //  <<-------  edit: 18/5/21
}

void loop() {
  if (!client.connected()) { //<------------------
    reconnect();
  }
  client.loop();

  String incString;

  if(Serial.available() > 0){
    incString = Serial.readStringUntil('$');

    Serial.println(incString);

      if(incString == "0") objDetected = "false";
      else if(incString == "1") objDetected = "true";
      else cap = incString;

    Serial.println("obj = " + objDetected + ", dist = " + cap);
  }
  
  long now = millis();
  if (now - lastMsg > 5000) {  //<<------------  ส่งทุกๆ 5 วินาที
    lastMsg = now;
    String data = "{\"data\": {\"distance\":" + cap + ", \"obj\":" + objDetected + "}}";   //  <<-------  edit: 18/5/21
    Serial.println(data);
    data.toCharArray(msg, (data.length() + 1));
    objDetected.toCharArray(msg2, (objDetected.length() +1));
    cap.toCharArray(msg3, (cap.length() +1));
    client.publish("@shadow/data/update", msg);   //<<---------------------------------------- ส่งdataขึ้น NETPIE
    client.publish("@msg/obj", msg2);  //<<--------------------ส่ง message(IR sensor) ให้หน้าเว็บเพื่ออัพเดท
    client.publish("@msg/cap", msg3);  //<<------------------ ส่งปริมาณขยะให้หน้าเว็บ  27/5/21
  }
}
