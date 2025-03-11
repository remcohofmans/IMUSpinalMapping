/*
 * WebServer.h
 * Handles the web server and WebSocket communication
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebSrv.h>
#include <LittleFS.h>
#include "SensorManager.h"
#include "FilterManager.h"

class WebServer {
public:
  WebServer(SensorManager* sensorMgr, FilterManager* filterMgr);
  
  // Initialize the web server and WebSocket
bool initialize(const char* ssid, const char* password, const char* username = nullptr);
  
  // Update orientation data to be sent to clients
  void updateOrientationData();
  
  // Server management methods
  void start();
  void stop();

private:
  SensorManager* sensorManager;
  FilterManager* filterManager;
  
  // Network configuration
  String ssid;
  String password;
  
  // Server components
  AsyncWebServer server;
  AsyncWebSocket webSocket;
  
  // Data to be sent to clients
  struct OrientationData {
    float roll[NO_OF_UNITS];
    float pitch[NO_OF_UNITS];
    float yaw[NO_OF_UNITS];
    bool active[NO_OF_UNITS];
  } orientationData;
  
  // Server event handlers
  void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                       AwsEventType type, void* arg, uint8_t* data, size_t len);
                       
  // Format data for JSON transmission
  String formatOrientationJSON();
};

#endif // WEB_SERVER_H
