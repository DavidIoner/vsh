#include <ESP8266WiFi.h> 
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
// ======================================================================
//Pinos---------------------
//SCL = D1
//SDA = D2

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


sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Constantes
char sensorId = 001;

//Timer---------------------
unsigned long lastTime = 0;  
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;  

//desvio giroscopio---------
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;


//Objetos-------------------
WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_MPU6050 mpu;



//Prototipos----------------
bool connectMQTT();

void setup(void)
{
  Serial.begin(9600);
//  initI2C();
//  addToJSON();
  initMPU();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  setWifi();

  client.setServer(mqtt_broker, mqtt_port);
//  client.setCallback(callback);
  mqttStatus = connectMQTT();
}



void loop() {
  static long long pooling = 0;
  if (mqttStatus) {
    client.loop();
    if (millis() > pooling + leitura) {
      pooling = millis();
        getAccReadings();
        getGyroReadings();
        defineState();
        addToJSON();
        printData();

      }
    }
}

void callback(char *topic, byte * payload, unsigned int length);


// Init MPU6050
void initMPU(){
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

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


void getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

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

void getAccReadings() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;

}

String getTemperature(){
  mpu.getEvent(&a, &g, &temp);
  temperature = temp.temperature;
  return String(temperature);
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







bool connectMQTT() {
  byte tentativa = 0;
  Serial.println("conectando ao broker");

  do {
    String client_id = "SENSOR01-";
    client_id += String(WiFi.macAddress());

    if (client.connect("nametest", mqtt_username, mqtt_password)) {
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


void callback(char *topic, byte * payload, unsigned int length) {
  Serial.println(topic);
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("---------------------------");
}
