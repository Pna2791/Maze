#ifndef OTA_UTILS_H
#define OTA_UTILS_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

#include <ElegantOTA.h>
#include <EEPROM.h>


WebServer server(80);


unsigned long ota_progress_millis = 0;
void onOTAStart() {
    // Log when OTA has started
    Serial.println("OTA update started!");
    // <Add your own code here>
}


void onOTAProgress(size_t current, size_t final) {
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000) {
        ota_progress_millis = millis();
        Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
}


void onOTAEnd(bool success) {
    // Log when OTA has finished
    if (success) {
        Serial.println("OTA update finished successfully!");
    } else {
        Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
}


void create_AP(String ssid="TrackerAP", String password="12345678"){
    WiFi.softAP(ssid, password);
    Serial.println("AP Started at 192.168.4.1");
}


void connect_to_WiFi(String ssid="B24.04", String password="baycuruoi"){
    // Define static IP settings
    IPAddress staticIP(192, 168, 1, 190); // Set your desired static IP
    IPAddress gateway(192, 168, 1, 1);    // Set your router's gateway IP
    IPAddress subnet(255, 255, 255, 0);   // Set subnet mask
    IPAddress dns(8, 8, 8, 8);            // Optional: Set DNS server (Google DNS)

    WiFi.mode(WIFI_STA);
    // Apply static IP
    WiFi.config(staticIP, gateway, subnet, dns); 
    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}


void START_OTA_MODE(){
    connect_to_WiFi();
    // create_AP();

    ElegantOTA.begin(&server);    // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    Serial.println("HTTP server started");
}


#endif
