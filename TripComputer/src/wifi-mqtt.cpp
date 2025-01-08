#include <WiFi.h>
#include <ArduinoOTA.h>
#include <wifi-mqtt.h>
#include <mysecrets.h>

IPAddress local_IP(192, 168, 30, 8);   // Desired static IP
IPAddress gateway(192, 168, 1, 1);      // Gateway address
IPAddress subnet(255, 255, 192, 0);     // Subnet mask
IPAddress primaryDNS(192, 168, 20, 1);       // Optional: Set primary DNS
IPAddress secondaryDNS(8, 8, 4, 4);     // Optional: Set secondary DNS

void StartWifi(){
    WiFi.config(local_IP, gateway, subnet);
    WiFi.hostname("esp32Trip");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
    }
}

void OTAsetup(){
// Initialize OTA
    ArduinoOTA.onStart([]() {
        //Serial.println("Start OTA Update");
    });
    ArduinoOTA.onEnd([]() {
        //Serial.println("\nEnd OTA Update");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        /*
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        */
    });
    ArduinoOTA.begin();
}
void OTAcheck()
{
     ArduinoOTA.handle();
}
WifiMQTT::WifiMQTT()
{
}