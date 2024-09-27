#include <WiFi.h>
#include <AsyncUDP.h>

const char* ssid = "your_ssid";
const char* password = "your_password";
const char* udpAddress = "your_server_ip";
const int udpPort = 3333;

AsyncUDP udp;

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  udp.listen(udpPort);
}

void onUdpPacket(AsyncUDPPacket packet) {
  Serial.println("Received packet from ");
  Serial.println(packet.remoteIP());
  Serial.println(packet.remotePort());
  Serial.println(packet.length());
  
  // Преобразование массива uint8_t* в строку char*
  char* data = reinterpret_cast<char*>(packet.data());
  Serial.println(data);
}



void loop() {
  udp.onPacket(&onUdpPacket);
  char message[] = "Hello, server!";
  
  // Преобразование строки char* в массив uint8_t*
  uint8_t* data = reinterpret_cast<uint8_t*>(message);
  udp.writeTo(data, strlen(message), udpAddress, udpPort);
  delay(1000);
}
