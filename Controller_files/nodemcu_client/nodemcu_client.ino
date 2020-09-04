//Include libraries
#include <ESP8266WiFi.h>

//Set ssid and password
const char* ssid = "J";
const char* pass = "Jaysable3";

//Server IP add and port
const char* addr = "172.20.10.3";
const uint16_t port = 65431;

//Temp variables
String data;
char prev_data;

void setup()
{
  //Start serial
  Serial.begin(115200);
  //Start WiFi
  WiFi.mode(WIFI_STA); //Force client mode
  WiFi.begin(ssid, pass);
  //Wait for connection
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.print("Connecting to ");
    Serial.println(ssid);
  }
  Serial.println("Connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop()
{
  //Create TCP Client object
  WiFiClient client;
  //Connect to server
  while (!client.connect(addr, port))
  {
    Serial.println("Wasted!");
  }
  Serial.println("Connected to server");
  //Infinite loop
  for(int i=0; i<3; i++)
  {
    //Get data from server
    while(client.available())
    {
      data = static_cast<String>(client.readString());
      Serial.println(data);
    }
//    if (client.connected()) {
//        client.println("hello from ESP8266");
//      }
    delay(1000);
  }
  client.stop();
  return;
}
