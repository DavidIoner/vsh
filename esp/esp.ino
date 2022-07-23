#include <ESP8266WiFi.h> 
#include <PubSubClient.h>
#include <Wire.h>
#include <ArduinoJson.h>
//#include "GY6050.h"       
// ======================================================================
//Pinos
const int sda_pin = D2; // I2C SDA
const int scl_pin = D1; // I2C SCL

//Memoria
const int MPU_ADDR =      0x68; // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I =      0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG =   0x1B; // registro de configuração do giroscópio
const int ACCEL_CONFIG =  0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT =    0x3B; // registro de leitura do eixo X do acelerômetro

//Wifi
const char* ssid     = "Orion*Fibra_David";
const char* password = "ioner004";

//MQTT Broker
const char *mqtt_broker = "52.67.193.122";
const char *topic = "sensor.downtime";
const char *mqtt_username = "david";
const char *mqtt_password = "JZTeVT5C";
const int mqtt_port = 1883;

//Variaveis
bool mqttStatus = 0;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
char sensorId = 001;
bool gyState;
float gyTotal;

//Constantes
const float gyOffSet = 0.03;

//Data
//const long gmtOffSet = -3 * 60 * 60; 


//Objetos
WiFiClient espClient;
PubSubClient client(espClient);

//Prototipos
bool connectMQTT();
void callback(char *topic, byte * payload, unsigned int length);

//JSON - função populateJSON()
// ======================================================================
//DynamicJsonDocument doc(1024);
//JsonObject& object = jsonBuffer.createObject();
//JsonObject& data = object.createNestedObject("data");
//   
//JsonObject& accel = data.createNestedObject("accel");
//JsonObject& temp = data.createNestedObject("temp");
//JsonObject& gyro = data.createNestedObject("gyro");
//   
//JsonObject& accelX = accel.createNestedObject("accelX");
//JsonObject& accelY = accel.createNestedObject("accelY");
//JsonObject& accelZ = accel.createNestedObject("accelZ");
// 
//JsonObject& gyroX = gyro.createNestedObject("gyroX");
//JsonObject& gyroY = gyro.createNestedObject("gyroY");
//JsonObject& gyroZ = gyro.createNestedObject("gyroZ");

// ======================================================================
/*
 * função que configura a I2C com os pinos desejados 
 * sda_pin -> D5
 * scl_pin -> D6
 */
void initI2C() 
{
  //Serial.println("---inside initI2C");
  Wire.begin(sda_pin, scl_pin);
}
 
/*
 * função que escreve um dado valor em um dado registro
 */
void writeRegMPU(int reg, int val)      //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                      // escreve o valor no registro
  Wire.endTransmission(true);           // termina a transmissão
}
 
/*
 * função que lê de um dado registro
 */
uint8_t readRegMPU(uint8_t reg)        // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}
 
/*
 * função que procura pelo sensor no endereço 0x68
 */
void findMPU(int mpu_addr)
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);
 
  if(data == 0)
  {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println("Dispositivo não encontrado!");
  }
}
 
/*
 * função que verifica se o sensor responde e se está ativo
 */
void checkMPU(int mpu_addr)
{
  findMPU(MPU_ADDR);
     
  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
   
  if(data == 104) 
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
 
    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
 
    if(data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    else Serial.println("MPU6050 em modo ACTIVE!"); 
  }
  else Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
}
 
/*
 * função de inicialização do sensor
 */
void initMPU()
{
  setSleepOff();
  setGyroScale();
  setAccelScale();
}
 
/* 
 *  função para configurar o sleep bit  
 */
void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}
 
/* função para configurar as escalas do giroscópio
   registro da escala do giroscópio: 0x1B[4:3]
   0 é 250°/s
 
    FS_SEL  Full Scale Range
      0        ± 250 °/s      0b00000000
      1        ± 500 °/s      0b00001000
      2        ± 1000 °/s     0b00010000
      3        ± 2000 °/s     0b00011000
*/
void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 3);
}
 
/* função para configurar as escalas do acelerômetro
   registro da escala do acelerômetro: 0x1C[4:3]
   0 é 250°/s
 
    AFS_SEL   Full Scale Range
      0           ± 2g            0b00000000
      1           ± 4g            0b00001000
      2           ± 8g            0b00010000
      3           ± 16g           0b00011000
*/
void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 3);
}
 
/* função que lê os dados 'crus'(raw data) do sensor
   são 14 bytes no total sendo eles 2 bytes para cada eixo e 2 bytes para temperatura:
 
  0x3B 59 ACCEL_XOUT[15:8]
  0x3C 60 ACCEL_XOUT[7:0]
  0x3D 61 ACCEL_YOUT[15:8]
  0x3E 62 ACCEL_YOUT[7:0]
  0x3F 63 ACCEL_ZOUT[15:8]
  0x40 64 ACCEL_ZOUT[7:0]
 
  0x41 65 TEMP_OUT[15:8]
  0x42 66 TEMP_OUT[7:0]
 
  0x43 67 GYRO_XOUT[15:8]
  0x44 68 GYRO_XOUT[7:0]
  0x45 69 GYRO_YOUT[15:8]
  0x46 70 GYRO_YOUT[7:0]
  0x47 71 GYRO_ZOUT[15:8]
  0x48 72 GYRO_ZOUT[7:0]
    
*/
void readRawMPU()
{  
  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                       // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
 
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();
 
  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();
 
  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 
 
  Serial.print("AcX = "); Serial.print(AcX / 2048);
  Serial.print(" | AcY = "); Serial.print(AcY / 2048);
  Serial.print(" | AcZ = "); Serial.print(AcZ / 2048);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
                                     
}

void defineState() {
  gyTotal = fabs(GyX) + fabs(GyY) + fabs(GyZ);
  if (gyTotal > gyOffSet) {
    gyState = 1;
  } else {
    gyState = 0;
  }
}

void addToJSON() {
  StaticJsonDocument<300> docAux;
  JsonObject reading = docAux.to<JsonObject>();
  reading["sensorId"] = sensorId;
  reading["date"] = "12342";
  reading["AcX"] = AcX;
  reading["AcY"] = AcY;
  reading["AcZ"] = AcZ;
  reading["State"] = gyState;
  
//  reading["qtyProduced"] = count;
//  mqttQueue.add(reading);
  char jsonOut[300];
  serializeJson(docAux, jsonOut);
//  client.publish(topic, jsonOut);
  reading.clear();
}

void setup(void)
{
  Serial.begin(115200);
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
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  mqttStatus = connectMQTT();
}


void loop() {
  static long long pooling = 0;
  if (mqttStatus) {
    client.loop();
    if (millis() > pooling + 1000) {
      pooling = millis();
      readRawMPU();    // lê os dados do sensor
      addToJSON();
      printData();
//      populateJSON();  // transforma os dados em formato JSON
//      client.publish(topic, docAux); // coloca um array dentro de uma string. array direto funciona?
    }
  }
}


bool connectMQTT() {
  byte tentativa = 0;


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
  } while (!client.connected() && tentativa < 20);

  if (tentativa < 20) {
    client.publish(topic, "{valor1, motivo1}");
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
