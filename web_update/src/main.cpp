#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
// #include <ArduinoOTA.h>
#include <Updater.h>
#include <ESP8266HTTPClient.h>
#include <wifiConnect.h>
#include <ESP_WiFiManager.h>


HTTPClient httpClient;
WiFiClient client;

uint32_t espID = ESP.getChipId();
int totalLength;
int currentLength = 0;

const char* url = "http://storage.googleapis.com/sensor-version/downtime/firmware3.bin";

      


void updateFirmware(uint8_t *data, size_t len, String filename){
  Update.write(data, len);
  currentLength += len;
  // Print dots while waiting for update to finish
  Serial.print('.');
  // if current length of written firmware is not equal to total firmware size, repeat
  if(currentLength != totalLength) return;
  Update.end(true);
  // APIlog((String)"Updated sucessfully from " + filename);
  Serial.printf("\nUpdate Success, Total Size: %u\nRebooting in 3s...\n", currentLength);
  delay(3000);
  // Restart ESP32 to see changes 
  ESP.restart();
}

void checkForUpdates() {

  Serial.println( "Checking for firmware updates." );

  httpClient.begin( client, url );
  int httpCode = httpClient.GET();

  if( httpCode == 200 ) {
    Serial.print("connected to ");
    Serial.println(url);
    // get length of document (is -1 when Server sends no Content-Length header)
    totalLength = httpClient.getSize();
    // transfer to local variable
    int len = totalLength;
    // this is required to start firmware update process
    Update.begin(len);
    Serial.printf("FW Size: %u\n",totalLength);
    // create buffer for read
    uint8_t buff[128] = { 0 };
    // get tcp stream
    WiFiClient * stream = httpClient.getStreamPtr();
    // read all data from server
    Serial.println("Updating firmware...");
    while(httpClient.connected() && (len > 0 || len == -1)) {
          // get available data size
          size_t size = stream->available();
          if(size) {
            // read up to 128 byte
            int c = stream->readBytes(buff, ((size > sizeof(buff)) ? sizeof(buff) : size));
            // pass to function
            updateFirmware(buff, c, "firmware.bin");
            if(len > 0) {
                len -= c;
            }
          }
          delay(1);
    }
  } else {
    Serial.print( "Firmware version check failed, got HTTP response code " );
    Serial.println( httpCode );
  }
  httpClient.end();
}

void setup() {
  Serial.begin(9600);
  wifiConnectBegin( "espID" , "123456789" );
  checkForUpdates();
  // Serial.println("AGORA VAI!!");

}

void loop() {

}