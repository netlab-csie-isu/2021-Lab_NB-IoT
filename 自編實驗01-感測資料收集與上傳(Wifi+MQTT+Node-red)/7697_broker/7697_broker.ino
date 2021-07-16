#include <LWiFi.h>
#include <PubSubClient.h>
#include "DHT.h"

#define DHTPIN A0
#define DHTTYPE DHT22 
DHT dht(DHTPIN, DHTTYPE);
#define LED_PIN 7

char ssid[] = "3715";
char password[] = "12345678";
char mqtt_server[] = "140.127.196.119"; //broker IP
char Pub_topic[] = "3715/temp_humid";   //publish topic
char Sub_topic[] = "3715/led";          //subscribe topic
char client_Id[] = "nbiot";       
char user_name[] = "iot";               //broker name
char user_password[] = "isuCSIE2021#";  //broker password

int status = WL_IDLE_STATUS;
WiFiClient mtclient;     
PubSubClient client(mtclient);

long lastMsg = 0;
char msg[50];
int value = 0;
// 儲存訊息的字串變數
String msgStr = "";
// 儲存字元陣列格式的訊息字串
char json[40];
//Initial value of time
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

void setup() {
    //Initialize serial and wait for port to open:
    Serial.begin(9600);
    //while (!Serial) {
         // wait for serial port to connect. Needed for native USB port only
    //}
    pinMode(LED_BUILTIN, OUTPUT);
    
    setup_wifi();
    dht.begin();    
    client.setServer(mqtt_server, 18314);
    client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  currentMillis = millis();
  //10second publish Temperature and Temperature
  if (currentMillis - previousMillis >= 10000) {
  // save the last time you blinked the LED
  previousMillis = currentMillis;
  dht_publish();
  }
}

//publish Temperature and Temperature
void dht_publish(){
  float h = dht.readHumidity();
  float t = dht.readTemperature();  
  Serial.print("Humidity: "); 
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: "); 
  Serial.print(t);
  Serial.println(" *C");  
  // 建立MQTT訊息（JSON格式的字串）
  msgStr = msgStr + "{\"temperature\":" + t + ",\"humidity\":" + h + "}";
  // 把String字串轉換成字元陣列格式
  msgStr.toCharArray(json, 40);
  // 發布MQTT主題與訊息
  client.publish(Pub_topic, json);
  // 清空MQTT訊息內容
  msgStr = ""; 
}

//setup Wifi
void setup_wifi() {  
   // attempt to connect to Wifi network:
   Serial.print("Attempting to connect to SSID: ");
   Serial.println(ssid);
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
     delay(500);
     Serial.print(".");
    }
    randomSeed(micros());
    Serial.println("Connected to wifi");
    printWifiStatus();
}

//print Wifi status
void printWifiStatus() {  
    // print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    // print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    // print the received signal strength:
    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
}
//MQTT subscribe
void callback(char* topic, byte* payload, unsigned int length) {   
   Serial.print("Message arrived [");
   Serial.print(Sub_topic);
   Serial.print("] ");

   for (int i=0;i<length;i++) {
   char receivedChar = (char)payload[i];
   Serial.print(receivedChar); // 打印mqtt接收到消息

   if (receivedChar == 't') {  // 收到消息是 't' 開啟LED灯
     digitalWrite(LED_PIN, HIGH);
   }
   if (receivedChar == 'f')    // 收到消息是 'f' 關閉LED灯
     digitalWrite(LED_PIN, LOW);
   }
   Serial.println();
}

//reconnect MQTT
void reconnect() {  
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = client_Id;
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(),user_name,user_password)) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      // ... and resubscribe
      client.subscribe(Sub_topic);
    } else {
      Serial.print("failed, ");
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
