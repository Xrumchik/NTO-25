#include <Wire.h> //I2C
#include "mcp3021.h" //вроде датчик протечки воды
#include <BH1750FVI.h> // датчик освещенности
#include "SparkFun_SGP30_Arduino_Library.h" //датчик летучих органических соединений и эквивалентной концентрации CO2 
#include <MGS_FR403.h> //датчик пламени
#include "MCP3221.h" //вроде датчик звука
#include <FastLED.h> //светодиодная лента
#include <HttpClient.h> //http запросы + api
#include "AsyncUDP.h" //асинхронная отправка udp пакетов

const char* ssid = "TP-Link_4F90"; //сеть wify
const char* password = "NTOContest202324"; //пароль от wify
const uint16_t port = 1024; //порт сайта

IPAddress addr(192, 168, 0, 234); //присваиваем esp ip

AsyncUDP udp; //экцепляр класса udp


bool s[27]; //???????????????????????????????????????????????????????????????????????????????????????????????????


////// Обработка udp пакетов - ОСОЗНАТЬ
int packet;
void stoi(uint8_t* a, size_t n){
  packet=0;
  for(int i=0;i<n;i++){
    packet*=10;
    packet+=(a[i]-'0');
  }
}
void parsePacket(AsyncUDPPacket packet)
{
  uint8_t* msg = packet.data();
  size_t len = packet.length();
  //Serial.write(msg, len);
  Serial.println("Пакет");
  //Serial.println(len);
  stoi(msg, len);
  //Serial.write(packet.data(), packet.length());
}
//////



//////лента и ее настройка определяя вывод, количество светодиодов, яркость, тип светодиода и порядок цветов
#define LED_PIN     A4
#define NUM_LEDS    18
#define BRIGHTNESS  64
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
#define UPDATES_PER_SECOND 100

CRGBPalette16 currentPalette;
TBlendType    currentBlending;

extern CRGBPalette16 myRedWhiteBluePalette;
extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
//////



////////// Константы для I2C шилда. адрес концентратора I2C, маску включения, канал по умолчанию и максимальный канал
#define I2C_HUB_ADDR        0x70
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/

byte ADDR = 0x4E; // 0x49 //????????????????????????????????????????????????????????????????????????????????????????
// (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
// 0x48 (000)...0x4F(111)
//////////////



///// калибровочные значения с АЦП
const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;
//////



//определяются экземпляры различных классов датчиков, включая датчик утечки воды, датчик качества воздуха, датчик пламени и датчик звука
MCP3021 mcp3021;
SGP30 mySensor;
MGS_FR403 Fire;
MCP3221 mcp3221(0x4E);
/////



///критические пороговые значения для CO2, TVOC и обнаружения пламени.
int ctitical_CO2 = 600; // здесь впишите пороговые значения, исходя из скетча MGS-CO30.ino
int critical_TVOC = 150;
int critical_FIRE = 500;
/////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////функция setup
void setup() {

  // Инициализация последовательного порта
  Serial.begin(115200);
  Serial.println();
  Serial.println();//много энтеров
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  ////



  // Инициализация I2C интерфейса
  Wire.begin();
  ///



  ///массив для управления свет.лентой 
  for(int i=0;i<27;i++){
    s[i]=0;
  }
  ////



  ///подключение к wify необходима для установления начального подключения к сети WiFi. 
  ///Однако после установления соединения модуль WiFi может находиться не в желаемом режиме (в данном случае в режиме станции).
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  ///Устанавливая режим WiFi на WIFI_STA и затем вызывая WiFi.begin() снова, код гарантирует, что модуль WiFi правильно настроен и соединение стабильное.



  /// переводим режим Wi-Fi в режим станции (WIFI_STA). Затем он повторно инициирует подключение, чтобы обеспечить стабильное соединение.
  Serial.println("Connected to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  ///



  // Ждём подключения WiFi
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  ///



  // Если удалось подключится по UDP
  if (udp.connect(addr, port)) {
    Serial.println("UDP подключён");
    // вызываем callback функцию при получении пакета
    udp.onPacket(parsePacket); 
  }
  // Если подключение не удалось
  else {
    Serial.println("UDP не подключён");
    // Входим в бесконечный цикл
    while (1) {
      delay(1000);
    }
  }
  ////



  //инициализация светодиодной ленты
  //delay( 3000 ); // power-up safety delay
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip ); //указание типа светодиода, pin-код, порядок цветов и количество светодиодов.
  FastLED.setBrightness(  BRIGHTNESS ); //установка значения яркости
  ///



  //инициализация датчика протечки воды
  setBusChannel(0x07); //установка канала шины I2C
  mcp3021.begin(ADDR); //инициализация
  ///



  //инициализация датчика газоанализатора на летучие органические соединения (3шт, тк 3 подъезда)
  setBusChannel(0x06);
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();

  setBusChannel(0x05);
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();

  setBusChannel(0x04);
  if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
  }
  mySensor.initAirQuality();
  /////



  //инициализация датчика пламени (3шт, тк 3 подъезда)
  setBusChannel(0x06);
  Fire.begin();

  setBusChannel(0x05);
  Fire.begin();

  setBusChannel(0x04);
  Fire.begin();
  ///



  ///делает всю светодиодную ленту черной
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] = CRGB::Black; FastLED.show();
  }
  /// 
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////функция loop
void loop() {
  if (udp.listen(port)) {
    // При получении пакета вызываем callback функцию
    udp.onPacket(parsePacket);
    if(packet!=0){
      Serial.println(packet);

    ///в зависимости от номера пакета подсвечивает определенный подъезд через светодиодную ленту
      if(packet==3){
        if(!s[packet]){
          for(int i=0;i<6;i++){
            leds[i] = CRGB::White; FastLED.show();
          }
          s[packet]=!s[packet];
        }
        else{
          for(int i=0;i<6;i++){
            leds[i] = CRGB::Black; FastLED.show();
          }
          s[packet]=!s[packet];
        }
      }
      if(packet==2){
        if(!s[packet]){
          for(int i=6;i<12;i++){
            leds[i] = CRGB::White; FastLED.show();
          }
          s[packet]=!s[packet];
        }
        else{
          for(int i=6;i<12;i++){
            leds[i] = CRGB::Black; FastLED.show();
          }
          s[packet]=!s[packet];
        }
      }
      if(packet==1){
        if(!s[packet]){
          for(int i=12;i<18;i++){
            leds[i] = CRGB::White; FastLED.show();
          }
          s[packet]=!s[packet];
        }
        else{
          for(int i=12;i<18;i++){
            leds[i] = CRGB::Black; FastLED.show();
          }
          s[packet]=!s[packet];
        }
      }
      packet=0;
    }
  }
  /////



  // Датчик воды.  читает значение датчика + преобразовывает его в проценты влажности 
  setBusChannel(0x07);
  float wat = mcp3021.readADC();
  float h = map(wat, air_value, water_value, moisture_0, moisture_100);
  // Вывод измеренных значений в терминал



  //Serial.println("Water level = " + String(h, 1) + " %");

  podezd1();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// Функция установки нужного выхода I2C - РАЗОБРАТЬСЯ
bool setBusChannel(uint8_t i2c_channel)
{
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(I2C_HUB_ADDR);
    //Wire.write(i2c_channel | EN_MASK); // для микросхемы PCA9547
    Wire.write(0x01 << i2c_channel); // Для микросхемы PW548A
    Wire.endTransmission();
    return true;
  }
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////функция подъезд 1
void podezd1(){
  //дaтчик ЛОС + измерение CO2
  setBusChannel(0x04);
  mySensor.measureAirQuality();
  int gas1=mySensor.CO2;
  ///



  if (gas1 > ctitical_CO2) {
    Serial.println("Превышен допустимый уровень углекислого газа! Подъезд 1");
  }
  String h = URL + "type=1" + "&" + "number=1" + "&" + "value=" + gas1 + "&bool=" + (gas1>ctitical_CO2);
  //Serial.println(h);
  HTTPClient http;
  http.begin(h);
  int httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    //Serial.println(payload);
  } else {
    Serial.println("Error on HTTP request");
  }
  http.end();
  /*if (mySensor.TVOC > critical_TVOC) {
    Serial.println("Превышен допустимый уровень летучих органических соединений! Подъезд 1");
  }*/

  //датчик пламени
  setBusChannel(0x04);
  Fire.get_ir_and_vis();
  int fire1=Fire.ir_data;
  if(Fire.ir_data>=critical_FIRE){
    Serial.print("Превышен допустимый уровень огня. Подъезд 1");
  }
  h = URL + "type=1" + "&" + "number=4" + "&" + "value=" + fire1 + "&bool=" + (fire1>critical_FIRE);
  http.begin(h);
  httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    //Serial.println(payload);
  } else {
    Serial.println("Error on HTTP request");
  }
  http.end();

  //датчик звука
  setBusChannel(0x04);
  float adc0 = mcp3221.getVoltage();
  if(adc0>=1800){
    Serial.println("Превышен уровень громкости. Подъезд 1");
  }
  h = URL + "type=1" + "&" + "number=7" + "&" + "value=" + adc0 + "&bool=" + (adc0>1800);
  http.begin(h);
  httpCode = http.GET();

  if (httpCode > 0) {
    String payload = http.getString();
    //Serial.println(payload);
  } else {
    Serial.println("Error on HTTP request");
  }
  http.end();
  //Serial.println("Sound level = " + String(adc0, 1));
}