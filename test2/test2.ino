#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
// #include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

HTTPClient httpClient;
WiFiClient client;
// versao atual do firmware
// const int FW_VERSION = 1244;
// Url do firmware
const char* fwUrlBase = "http://storage.googleapis.com";

const char* ssid = "Orion*Fibra_David";
const char* password = "ioner004";

void setWifi(){
  Serial.println();      
  WiFi.begin(ssid, password); //Inicia WiFi 
  delay(500);
  Serial.print("Conectando"); 
  while (WiFi.status() != WL_CONNECTED) // Tentando conectar na Rede WiFi
  {
    delay(500);
    Serial.print(".");
   }
  Serial.println();
  
  Serial.print("Conectado | Endereço IP: "); 
  Serial.println(WiFi.localIP()); //Imprime o Endereço IP Local do ESP8266
}

void checkForUpdates() {
  String mac = WiFi.macAddress();
  String fwURL = String( fwUrlBase );
  fwURL.concat( mac );
  // String fwVersionURL = fwURL;
  // fwVersionURL.concat( ".version" );
  Serial.println( "Checking for firmware updates." );
  Serial.print( "MAC address: " );
  // Serial.println( mac );
  // Serial.print( "Firmware version URL: " );
  // Serial.println( fwVersionURL );


  // httpClient.begin( client, fwVersionURL );
  // apiClient.begin("https://storage.googleapis.com/sensor-version/downtime/firmware.bin");
  httpClient.begin( client, "http://storage.googleapis.com/sensor-version/downtime/firmware.bin" );
  int httpCode = httpClient.GET();
  if( httpCode == 200 ) {
    String newFWVersion = httpClient.getString();
    Serial.println(newFWVersion);

    // Serial.print( "Current firmware version: " );
    // Serial.println( FW_VERSION );
    // Serial.print( "Available firmware version: " );
    // Serial.println( newFWVersion );

    // int newVersion = newFWVersion.toInt();

    // if( newVersion > FW_VERSION ) 
    
    if( 1==1 ){
      Serial.println( "Preparing to update" );

      // String fwImageURL = fwURL;
      // fwImageURL.concat( ".bin" );
      // update(client, host (httpserver), port, uri)
      t_httpUpdate_return ret = ESPhttpUpdate.update( client, "https://storage.googleapis.com/sensor-version/downtime", 80, "/firmware.bin" );

      switch(ret) {
        case HTTP_UPDATE_FAILED:
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
          break;

        case HTTP_UPDATE_NO_UPDATES:
          Serial.println("HTTP_UPDATE_NO_UPDATES");
          break;
        // anti bug
        default:
          Serial.println("some error (?)");
          break;
      }
    }
    else {
      Serial.println( "Already on latest version" );
    }
  }
  else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
  }
  httpClient.end();
}


void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  setWifi();
  checkForUpdates();
}

void loop() {

}
