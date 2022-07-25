#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
// #include <ArduinoOTA.h>
#include <pubSubCLient.h>
#include <Updater.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//Wifi----------------------
const char* ssid     = "Orion*Fibra_David";
const char* password = "ioner004";

//MQTT Broker---------------
const char *mqtt_broker = "52.67.193.122";
const char *topic = "sensor.downtime";
const char *mqtt_username = "david";
const char *mqtt_password = "JZTeVT5C";
const int mqtt_port = 1883;

//Variaveis-----------------
bool mqttStatus = 0;
//int leitura = 1/30 * 1000;
int leitura = 500;
bool engineState;
int stateCount;
float gyTotal;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//desvio giroscopio---------
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

//Objetos-------------------
sensors_event_t a, g, temp;
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;
HTTPClient httpClient;
HTTPClient apiClient;

String espId;

uint32_t espFlashId = ESP.getFlashChipId();
int totalLength;
int currentLength = 0;
String sensorId;
int MAX_REQ_ATTEMPT = 5;
String API_SERVER = "http://52.67.193.122";
int API_PORT = 3000;
const char* url = "http://storage.googleapis.com/sensor-version/downtime/";

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

String router(String subroute){
  return API_SERVER + (String)":" + API_PORT + subroute;
}

void generateSensorId() {
  char espIdAux[12];
  snprintf(espIdAux, 12, "%X%X", (uint32_t)ESP.getFlashChipId(), (uint32_t)ESP.getChipId());
  espId = espIdAux;
}

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
  Serial.println(espId);
  String route = "/check-update?id=" + espId + "&downtime=true";
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
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void getReadings(){
  mpu.getEvent(&a, &g, &temp);

  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

  temperature = temp.temperature;

  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/50.00;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  }
}

void defineState() {
  gyTotal = abs(gyroX) + abs(gyroY) + abs(gyroZ);
  if (gyTotal > 0) {
      stateCount ++;
  } else {
    stateCount = 0;
    engineState = 0; 
  }
  if (stateCount > 4) { //quantidade de leituras para considerar o motor como ligado
      engineState = 1;
  } 
}

void addToJSON() {
  StaticJsonDocument<300> docAux;
  JsonObject reading = docAux.to<JsonObject>();
  reading["sensorId"] = sensorId;
  reading["date"] = "12342";
  reading["accX"] = accX;
  reading["accY"] = accY;
  reading["accZ"] = accZ;

  reading["gyroX"] = gyroX;
  reading["gyroY"] = gyroY;
  reading["gyroZ"] = gyroZ;

  reading["engineState"] = engineState;

//  mqttQueue.add(reading);
  char jsonOut[300];
  serializeJson(docAux, jsonOut);
  client.publish(topic, jsonOut);
  reading.clear();
}

void printData() {
  
      /* Print out the values */
      Serial.print("Acceleration X: ");
      Serial.print(a.acceleration.x);
      Serial.print(", Y: ");
      Serial.print(a.acceleration.y);
      Serial.print(", Z: ");
      Serial.print(a.acceleration.z);
      Serial.println(" m/s^2");
    
      Serial.print("Rotation X: ");
      Serial.print(g.gyro.x);
      Serial.print(", Y: ");
      Serial.print(g.gyro.y);
      Serial.print(", Z: ");
      Serial.print(g.gyro.z);
      Serial.println(" rad/s");
    
      Serial.print("Temperature: ");
      Serial.print(temp.temperature);
      Serial.println(" degC");
    
      Serial.println("");
}

bool connectMQTT() {
  byte tentativa = 0;
  Serial.println("conectando ao broker");

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
  generateSensorId();
  checkForUpdates();
  initMPU();
  client.setServer(mqtt_broker, mqtt_port);
  //  client.setCallback(callback);
  mqttStatus = connectMQTT();

  // Serial.println("AGORA VAI!!");

}

void loop() {
  static long long pooling = 0;
  if (mqttStatus) {
    client.loop();
    if (millis() > pooling + leitura) {
      pooling = millis();
        getReadings();
        defineState();
        addToJSON();
        printData();
      }
    }
}