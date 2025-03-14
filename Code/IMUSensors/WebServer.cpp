/*
 * WebServer.cpp
 * Implementation of the WebServer class
 */

#include "WebServer.h"
#include <ArduinoJson.h>

// Constructor
WebServer::WebServer(SensorManager* sensorMgr, FilterManager* filterMgr)
  : sensorManager(sensorMgr),
    filterManager(filterMgr),
    server(80),
    webSocket("/ws")
{
  // Set up WebSocket event handler using a standard lambda
  webSocket.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len) {
    this->onWebSocketEvent(server, client, type, arg, data, len);
  });
  
  // Initialize orientation data
  for (int i = 0; i < NO_OF_UNITS; i++) {
    orientationData.roll[i] = 0;
    orientationData.pitch[i] = 0;
    orientationData.yaw[i] = 0;
    orientationData.active[i] = false;
  }
}

bool WebServer::initialize(const char* ssid, const char* password, const char* hostname) {
  // Store credentials
  this->ssid = ssid;
  this->password = password;
  
  // Connect to WiFi in station mode
  Serial.println("Connecting to WiFi network: " + String(ssid));
  
  WiFi.mode(WIFI_STA);
  
  // Set hostname if provided
  if (hostname != nullptr && strlen(hostname) > 0) {
    WiFi.setHostname(hostname);
  }
  
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) { // 10 second timeout
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nFailed to connect to WiFi network");
    return false;
  }
  
  Serial.println("\nConnected to WiFi network");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Add WebSocket to server
  server.addHandler(&webSocket);

  // Add CORS headers to all responses
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");
  
  // Define routes for static files
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  
  // Serve static files from LittleFS
  server.serveStatic("/", LittleFS, "/");
  
  return true;
}

void WebServer::start() {
  server.begin();
  Serial.println("HTTP server started");
}

void WebServer::stop() {
  webSocket.closeAll();
  // server.end(); // Note: AsyncWebServer doesn't have an end() method
}

void WebServer::updateOrientationData() {
  for (int i = 0; i < NO_OF_UNITS; i++) {
    orientationData.active[i] = sensorManager->isSensorActive(i);
    
    if (orientationData.active[i]) {
      float roll, pitch, yaw;
      filterManager->getOrientation(i, roll, pitch, yaw);
      
      orientationData.roll[i] = roll;
      orientationData.pitch[i] = pitch;
      orientationData.yaw[i] = yaw;
    }
  }
  
  // Send updated data to all connected clients
  String jsonString = formatOrientationJSON();
  webSocket.textAll(jsonString);
}

String WebServer::formatOrientationJSON() {
  String json = "{\"sensors\":[";
  
  bool firstSensor = true;
  for (int i = 0; i < NO_OF_UNITS; i++) {
    if (orientationData.active[i]) {
      if (!firstSensor) {
        json += ",";
      }
      firstSensor = false;
      
      json += "{\"id\":";
      json += i;
      json += ",\"orientation\":{\"roll\":";
      json += orientationData.roll[i];
      json += ",\"pitch\":";
      json += orientationData.pitch[i];
      json += ",\"yaw\":";
      json += orientationData.yaw[i];
      json += "}}";
    }
  }
  
  json += "]}";
  return json;
}

void WebServer::onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                                AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      // Handle incoming data (if needed)
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}