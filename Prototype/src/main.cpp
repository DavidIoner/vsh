#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <pubSubCLient.h>
#include <Updater.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NTPClient.h>
#include <ESPDateTime.h>


//Wifi----------------------
const char* ssid     = "Orion*Fibra_David";
// 
// WS-TISBC
// WS-SP6
const char* password = "ioner004";
// s@30@ltr&f9wss
// s@30@ltr&f9ws
// IPAddress local_IP(192, 168, 3, 254); 
// Set your Gateway IP address
// IPAddress gateway(192, 168, 1, 254);

// IPAddress subnet(255, 255, 0, 0);
// IPAddress primaryDNS(8, 8, 8, 8);   //optional
// IPAddress secondaryDNS(8, 8, 4, 4); //optional  

//MQTT Broker---------------
const char *mqtt_broker = "177.71.187.19";
const char *topic = "sensor.downtime";
const char *mqtt_username = "david";
const char *mqtt_password = "JZTeVT5C";
const int mqtt_port = 1883;

//Variaveis-----------------
bool mqttStatus = 0;

float vibration;
float leituras = 0;
float spectrum;
//int leitura = 1/30 * 1000;
bool engineState;
int stateCount;
float gyTotal;

float gyMedia;
float gyMediaTotal;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

float gyroX_temp;
float gyroY_temp;
float gyroZ_temp;

//desvio giroscopio---------
float gyroXerror = 0.05;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

//tempo---------------------
unsigned long lastMs = 0;
unsigned long ms = millis();

static long long timer = 250;

// static long long pooling = 0;
// static long long pooling2 = 0;


//Objetos-------------------
sensors_event_t a, g, temp;
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;

HTTPClient httpClient;
HTTPClient apiClient;


String espId;

int totalLength;
int currentLength = 0;
String sensorId;
int MAX_REQ_ATTEMPT = 5;
String API_SERVER = "http://177.71.187.19";
int API_PORT = 3000;
const char* url = "http://storage.googleapis.com/sensor-version/downtime/";

void setWifi(){
  Serial.println();    

  // if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
  //   Serial.println("STA Failed to configure");
  // }else{Serial.println("configurou");}
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

void setupDateTime() {

  // DateTime.setServer("asia.pool.ntp.org");
  // DateTime.begin(15 * 1000);
  // from
  /** changed from 0.2.x **/
  DateTime.setTimeZone("UTC-0");
  // this method config ntp and wait for time sync
  // default timeout is 10 seconds
  DateTime.begin(/* timeout param */);
  if (!DateTime.isTimeValid()) {
    Serial.println("Failed to get time from server.");
  }
}


String router(String subroute){
  return API_SERVER + (String)":" + API_PORT + subroute;
}

void APIlog(String msg){
  String route = "/sensor-log";
  StaticJsonDocument<1024> docAux;
  JsonObject reading = docAux.to<JsonObject>();
  String jsonAux;

  reading["id"] = sensorId;
  // reading["date"] = getCurrentDateTime();
  reading["log"] = msg;

  serializeJson(reading, jsonAux);

  apiClient.begin(espClient , router(route));
  apiClient.addHeader("Content-Type", "application/json");     

  int res = apiClient.POST(jsonAux);
  apiClient.end();
  reading.clear();
}

void generateSensorId() {
  char espIdAux[12];
  snprintf(espIdAux, 12, "%X%X", (uint32_t)ESP.getFlashChipId(), (uint32_t)ESP.getChipId());
  sensorId = espIdAux;
}

void updateFirmware(uint8_t *data, size_t len, String filename){
  Update.write(data, len);
  currentLength += len;
  // Print dots while waiting for update to finish
  Serial.print('.');
  // if current length of written firmware is not equal to total firmware size, repeat
  if(currentLength != totalLength) return;
  Update.end(true);
  APIlog((String)"Updated sucessfully from " + filename);
  Serial.printf("\nUpdate Success, Total Size: %u\nRebooting in 3s...\n", currentLength);
  delay(3000);
  // Restart ESP32 to see changes 
  ESP.restart();
}

void checkForUpdates() {
  Serial.println(sensorId);
  String route = "/check-update?id=" + sensorId + "&downtime=true";
  Serial.println(route);
  Serial.println(router(route));

  

  apiClient.begin(espClient , router(route));

  int statusCode = HTTP_CODE_BAD_REQUEST;
  int attempt = 0;

  while((statusCode != HTTP_CODE_OK) && (attempt < MAX_REQ_ATTEMPT)) {
    attempt++;
    statusCode = apiClient.GET();
  }
  if(statusCode != HTTP_CODE_OK) {

    // ESP.restart();
  }
  String filename = apiClient.getString();
  apiClient.end();

  if(filename == ""){
    Serial.println("Already up to date.");
    return;
  }

  String urlf = url + filename;

  Serial.println( "Avaliable." );

  httpClient.begin( espClient, urlf );

  int httpCode = httpClient.GET();

  if( httpCode == 200 ) {
    Serial.print("connected to ");
    Serial.println(urlf);
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
            updateFirmware(buff, c, filename);
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

void initMPU(){
  // 2, 4, 8, 16
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // 250, 500, 1000, 2000
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  // 260, 184, 94, 44, 21, 10, 5
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}



void getReadings(){
  mpu.getEvent(&a, &g, &temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  temperature = temp.temperature;

  // gyroX = g.gyro.x;
  // gyroY = g.gyro.y;
  // gyroZ = g.gyro.z;
  

  gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) >= gyroXerror)  {
    gyroX = gyroX_temp - gyroXerror;
  } else{gyroX = 0;}
  
  gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) >= gyroYerror) {
    gyroY = gyroY_temp - gyroYerror;
  } else{gyroY = 0;}

  gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) >= gyroZerror) {
    gyroZ = gyroZ_temp - gyroZerror;
  } else{gyroZ = 0;}
}

void defineState() {
  gyTotal = abs(gyroX) + abs(gyroY) + abs(gyroZ);
  if (gyTotal > 0) {
      stateCount ++;
  } else if (stateCount){
    stateCount --;
    engineState = 0; 
  }
  if (stateCount > 4) { //quantidade de leituras para considerar o motor como ligado
      engineState = 1;
  } 
}

float toVibrationSpectrum(){
  gyMedia = 0;
  // 5 leituras
  
  getReadings();
  gyTotal = abs(gyroX) + abs(gyroY) + abs(gyroZ);
  gyMedia += gyTotal;

  return gyMedia;
}


void addToJSON() {
  StaticJsonDocument<300> docAux;
  JsonObject reading = docAux.to<JsonObject>();
  reading["id"] = sensorId;
  reading["t"] = DateTime.toISOString();
  reading["v"] = spectrum;
  // reading["a"] = accX;
  // reading["b"] = accY;
  // reading["c"] = accZ;

  // reading["x"] = gyroX;
  // reading["y"] = gyroY;
  // reading["z"] = gyroZ;

  char jsonOut[300];
  serializeJson(docAux, jsonOut);
  client.publish(topic, jsonOut);
  reading.clear();
}

void printData() {
  
      /* Print out the values */
      
      // Serial.print(a.acceleration.x);
      // Serial.println(" ");
      // Serial.print(a.acceleration.y);
      // Serial.println(" ");
      // Serial.print(a.acceleration.z);
      // Serial.println(" ");
    
      Serial.print(gyroX);
      Serial.print(" ");
      Serial.print(gyroY);
      Serial.print(" ");
      Serial.print(gyroZ);
      Serial.println(" ");
    
      // Serial.print("Temperature: ");
      // Serial.print(temp.temperature);
      // Serial.println(" degC");
    
      // Serial.println("");
}

bool connectMQTT() {
  byte tentativa = 0;
  Serial.println("conectando ao broker");
  client.setServer(mqtt_broker, mqtt_port);

  do {
    String client_id = "SENSOR01-";
    client_id += String(WiFi.macAddress());

    if (client.connect(espId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Exito na conexao");
      Serial.printf("Cliente %s conectado ao broker\n",client_id.c_str());  
    } else{
      Serial.print("Falha ao conectar: ");
      Serial.print(client.state());
      Serial.println();
      Serial.print("Tentativa: ");
      delay(2000);
    }
    tentativa++;
  } while (!client.connected() && tentativa < 5);

  if (tentativa < 5) {
    return 1;
  } else {
    Serial.println("nao conectado");
    return 0;
  }
}

void setup() {
  Serial.begin(9600);
  // wifiConnectBegin( "espID" , "123456789" );
  setWifi();
  setupDateTime();
  generateSensorId();
  // checkForUpdates();
  initMPU();
  //  client.setCallback(callback);
  mqttStatus = connectMQTT();
  if (mqttStatus == 0) {
    ESP.reset();
  }

  // Serial.println("AGORA VAI!!");

}

void loop() {
  static long long pooling = 0;
  static long long pooling2 = 0;
  
  // Serial.println(pooling);
  if (mqttStatus) {
    client.loop();
    if (millis() > pooling + timer) {
      pooling = millis(); 

      // faz a leitura e retorna um float de vibracao
      vibration += toVibrationSpectrum();
      Serial.println(vibration);
      leituras += 1;
      if (millis() >= pooling2 + 1000){
        pooling2 = millis();
        // faz a media das leituras
        spectrum = vibration/leituras;
        addToJSON();
        vibration = 0;
        leituras = 0;
      }

      

      printData();


        if (WiFi.status() != WL_CONNECTED){
          ESP.reset();
        }
        }
    }
  }
