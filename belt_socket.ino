#include <DNSServer.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include "ESPAsyncWebSrv.h"

#include <WebSocketsServer_Generic.h>

const char* ssid = "dlink";
const char* password = "";
int motor_pins[] = {13, 27, 26, 25, 33, 32, 18, 19};


AsyncWebServer server(270);
AsyncWebSocket ws("/ws");

void setup() 
{
  Serial.begin(115200);
  Serial.println();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.print("Connected to WiFi. IP address: ");
  Serial.println(WiFi.localIP());
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);

  server.begin();
  for(int i = 0 ; i < 8 ; i++)
  {
    pinMode(motor_pins[i], OUTPUT);
    digitalWrite(motor_pins[i], LOW);
  }
  for(int i = 0 ; i < 8 ; i++)
  {
    digitalWrite(motor_pins[i], HIGH);
    delay(1000);
    digitalWrite(motor_pins[i], LOW);
  }
}

void loop() {
  // Your ESP32 code logic here
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->opcode == WS_TEXT) {
      String message = "";
      for (size_t i = 0; i < len; i++) {
        message += (char)data[i];
      }
      Serial.println("Received message: " + message);

      int t = 500;

      for(int i = 0 ; i < 8 ; i++)
      {
        if(message[i] == '1')
        {
          digitalWrite(motor_pins[i], HIGH);  
          delay(t);         
          digitalWrite(motor_pins[i], LOW);         
        }
      }
      // Send a response back to the WebSocket client
      client->text("Message received by ESP32: " + message);
    }
  }
}
