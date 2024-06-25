#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

const char* ssid = "Redminib";
const char* password = "aryavarta";
String webpage = "<!DOCTYPE html><html><head><title>HyperHorizon</title></head><body style='background-color: #000000;'><span style='color: #FFFFFF;'><h1>HyperHorizon</h1><p>Set Frequency: <span id='freq'>-</span></p><p>Sweep Time: <span id='sweep'>-</span></p><p>Received Latitude: <span id='rec_lat'>-</span></p><p>Received Longitude: <span id='rec_long'>-</span></p><p>DVL Status: <span id='dvl_stat'>-</span></p><p>UVL Status: <span id='uvl_stat'>-</span></p><p>GPS Status: <span id='gps_stat'>-</span></p><p>PA Status: <span id='pa_stat'>-</span></p><p>Roll: <span id='roll'>-</span></p><p>Pitch: <span id='pitch'>-</span></p><p>Yaw: <span id='yaw'>-</span></p><p>Base Station Latitude: <span id='base_lat'>-</span></p><p>Base Station Longitude: <span id='base_long'>-</span></p><p><button type='button' id='BTN_SEND_BACK'>Send info to ESP32</button></p><p><button type='button' id='BTN_SEND_PACKET'>Send Packet</button></p><p> <button type='button' id='BTN_SET_FREQ'>Set Frequency</button> <input type='text' id='freq_input' style='display:none;' placeholder='Enter Frequency'> <button type='button' id='BTN_SEND_FREQ' style='display:none;'>Send Frequency</button></p><p> <button type='button' id='BTN_SET_SWEEP'>Set Sweep Time</button> <input type='text' id='sweep_input' style='display:none;' placeholder='Enter Sweep Time'> <button type='button' id='BTN_SEND_SWEEP' style='display:none;'>Send Sweep Time</button></p><p> <button type='button' id='BTN_RTB'>Return to Base</button> <button type='button' id='BTN_CONFIRM_RTB' style='display:none;'>Confirm Return to Base</button></p><p> <button type='button' id='BTN_ABORT'>Abort Mission</button> <button type='button' id='BTN_CONFIRM_ABORT' style='display:none;'>Confirm Abort Mission</button></p><p> <button type='button' id='BTN_ADD_WAYPOINT'>Add Waypoint</button> <input type='text' id='waypoint_lat' style='display:none;' placeholder='Enter Waypoint Latitude'> <input type='text' id='waypoint_long' style='display:none;' placeholder='Enter Waypoint Longitude'> <button type='button' id='BTN_SEND_WAYPOINT' style='display:none;'>Send Waypoint</button></p></span></body><script> var Socket; document.getElementById('BTN_SEND_BACK').addEventListener('click', button_send_back); document.getElementById('BTN_SEND_PACKET').addEventListener('click', button_send_packet); document.getElementById('BTN_SET_FREQ').addEventListener('click', button_show_freq_input); document.getElementById('BTN_SEND_FREQ').addEventListener('click', button_send_freq); document.getElementById('BTN_SET_SWEEP').addEventListener('click', button_show_sweep_input); document.getElementById('BTN_SEND_SWEEP').addEventListener('click', button_send_sweep); document.getElementById('BTN_RTB').addEventListener('click', button_show_confirm_rtb); document.getElementById('BTN_CONFIRM_RTB').addEventListener('click', button_confirm_rtb); document.getElementById('BTN_ABORT').addEventListener('click', button_show_confirm_abort); document.getElementById('BTN_CONFIRM_ABORT').addEventListener('click', button_confirm_abort); document.getElementById('BTN_ADD_WAYPOINT').addEventListener('click', button_show_waypoint_input); document.getElementById('BTN_SEND_WAYPOINT').addEventListener('click', button_send_waypoint); function init() { Socket = new WebSocket('ws://' + window.location.hostname + ':81/'); Socket.onmessage = function(event) { processCommand(event); }; } function button_send_back() { var msg = { brand: 'Gibson', type: 'Les Paul Studio', year: 2010, color: 'white' }; Socket.send(JSON.stringify(msg)); } function button_send_packet() { Socket.send('S'); } function button_show_freq_input() { document.getElementById('freq_input').style.display = 'inline'; document.getElementById('BTN_SEND_FREQ').style.display = 'inline'; } function button_send_freq() { var freq_value = document.getElementById('freq_input').value; Socket.send(JSON.stringify({freq: freq_value})); } function button_show_sweep_input() { document.getElementById('sweep_input').style.display = 'inline'; document.getElementById('BTN_SEND_SWEEP').style.display = 'inline'; } function button_send_sweep() { var sweep_value = document.getElementById('sweep_input').value; Socket.send(JSON.stringify({sweep: sweep_value})); } function button_show_confirm_rtb() { document.getElementById('BTN_CONFIRM_RTB').style.display = 'inline'; } function button_confirm_rtb() { Socket.send('RTB'); } function button_show_confirm_abort() { document.getElementById('BTN_CONFIRM_ABORT').style.display = 'inline'; } function button_confirm_abort() { Socket.send('ABORT'); } function button_show_waypoint_input() { document.getElementById('waypoint_lat').style.display = 'inline'; document.getElementById('waypoint_long').style.display = 'inline'; document.getElementById('BTN_SEND_WAYPOINT').style.display = 'inline'; } function button_send_waypoint() { var waypoint_lat = document.getElementById('waypoint_lat').value; var waypoint_long = document.getElementById('waypoint_long').value; Socket.send(JSON.stringify({waypoint_lat: waypoint_lat, waypoint_long: waypoint_long})); } function processCommand(event) { var obj = JSON.parse(event.data); document.getElementById('freq').innerHTML = obj.freq; document.getElementById('sweep').innerHTML = obj.sweep; document.getElementById('rec_lat').innerHTML = obj.rec_lat; document.getElementById('rec_long').innerHTML = obj.rec_long; document.getElementById('dvl_stat').innerHTML = obj.dvl_stat; document.getElementById('uvl_stat').innerHTML = obj.uvl_stat; document.getElementById('gps_stat').innerHTML = obj.gps_stat; document.getElementById('pa_stat').innerHTML = obj.pa_stat; document.getElementById('roll').innerHTML = obj.roll; document.getElementById('pitch').innerHTML = obj.pitch; document.getElementById('yaw').innerHTML = obj.yaw; document.getElementById('base_lat').innerHTML = obj.base_lat; document.getElementById('base_long').innerHTML = obj.base_long; } window.onload = function(event) { init(); }</script></html>";
int interval = 1000;
unsigned long previousMillis = 0;
String freq = "0";
String sweep = "0";

WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("Establishing connection to WiFi with SSID: " + String(ssid));
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.print("Connected to network with IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", []() {
    server.send(200, "text/html", webpage);
  });
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  server.handleClient();
  webSocket.loop();

  unsigned long now = millis();
  if ((unsigned long)(now - previousMillis) > interval) {
    String jsonString = "";
    StaticJsonDocument<400> doc;
    JsonObject object = doc.to<JsonObject>();

    object["freq"] = freq;
    object["sweep"] = sweep;
    object["rec_lat"] = random(0, 9000000) / 100000.0;
    object["rec_long"] = random(0, 18000000) / 100000.0; 
    object["dvl_stat"] = random(0, 2); 
    object["uvl_stat"] = random(0, 2); 
    object["gps_stat"] = random(0, 2); 
    object["pa_stat"] = random(0, 2); 
    object["roll"] = random(-180, 180);
    object["pitch"] = random(-90, 90); 
    object["yaw"] = random(0, 360); 

    serializeJson(doc, jsonString);
    Serial.println(jsonString);
    webSocket.broadcastTXT(jsonString);

    previousMillis = now;
  }
}

void webSocketEvent(byte num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.println("Client " + String(num) + " disconnected");
      break;
    case WStype_CONNECTED:
      Serial.println("Client " + String(num) + " connected");
      break;
    case WStype_TEXT: {
      StaticJsonDocument<200> doc;
      DeserializationError error = deserializeJson(doc, payload, length);
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        return;
      }

      if (doc.containsKey("freq")) {
        int freq_value = doc["freq"];
        Serial.println("Received frequency: " + String(freq_value));
        // Handle frequency setting logic
      } else if (doc.containsKey("sweep")) {
        int sweep_value = doc["sweep"];
        Serial.println("Received sweep time: " + String(sweep_value));
        // Handle sweep time setting logic
      } else if (doc.containsKey("waypoint_lat") && doc.containsKey("waypoint_long")) {
        float waypoint_lat = doc["waypoint_lat"];
        float waypoint_long = doc["waypoint_long"];
        Serial.println("Received waypoint: Lat " + String(waypoint_lat) + ", Long " + String(waypoint_long));
        // Handle waypoint setting logic
      } else {
        String msg = String((char*) payload);
        Serial.println("Message from user: " + msg);
        if (msg == "RTB") {
          Serial.println("Received Return to Base command");
          // Handle Return to Base logic
        } else if (msg == "ABORT") {
          Serial.println("Received Abort Mission command");
          // Handle Abort Mission logic
        }
      }
      break;
    }
    default:
      break;
  }
}
