#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <string.h>
#include <ArduinoJson.h>

#define SDA_PIN 23
#define SCL_PIN 22

#define OLED_ADDR 0x3C
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define BASE_PIXELS_PER_CHAR 6

#define DEFAULT_SERVER_ADDR "play.cubecraft.net"
#define SERVER_ADDR_SIZE 64
#define DEFAULT_SERVER_PORT 25565
#define MAX_PACKET_SIZE 16000 // Carefull with this one, as the packet size is proportional to the number of players in the server

#define SSID_SIZE 32
#define PASSWD_SIZE 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

bool online;

// My own implementation, I had some problems with the \r character so I decided to make this
char wifiSSID[SSID_SIZE];
ssize_t ssidSize = 0;
char wifiPasswd[PASSWD_SIZE];
ssize_t passwdSize = 0;

typedef enum Tstate
{
  HANDSHAKE,
  SERVER_STATUS,
  PING,
  END
} state_t;
state_t state;

/**
 * Struct that represents the status of a minecraft server. Commented fields are not used
 *
 *
 */
struct status_t
{
  // char motd[512];
  // unsigned char icon[64 * 64];
  bool enforcesSecureShat;
  char versionName[32];
  // unsigned int protocolVersion;
  int onlinePlayers;
  int maxPlayers;
  int ping;
};

WiFiMulti wifiMulti;
WiFiClient client;

char *buffer;
char serverAddr[SERVER_ADDR_SIZE];
unsigned short serverPort;

ssize_t serialReceiveBytesUntil(char *buffer, const ssize_t size, const char until = '\n');

uint32_t clientReceiveVarInt();

ssize_t writeIntToVarInt(char **buffer, unsigned int number);

void serializeUShort(char **buffer, unsigned short num);

ssize_t sendHandshake(char *buffer);

ssize_t sendStatusRequest(char *buffer);

ssize_t receiveServerStatus(struct status_t &status);

void showServerStatus(const struct status_t &status);

ssize_t sendPing(char *buffer, long long check);

ssize_t receivePong(char *buffer, struct status_t &status, unsigned long timer, long long check);

void printOnDisplay(uint8_t size, uint16_t color, int16_t x, int16_t y, String text, bool cp437 = true);

void displayTitle();

int16_t centerXText(int sizeFactor, String text);

bool textOverflows(int sizeFactor, String text, uint16_t x);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR))
  {
    Serial.println("SSD1306 display not found");
    for (;;)
      ;
  }

  display.clearDisplay();
  display.display();

  bool connected = false;
  bool passwIntroduced = false;

  while (!connected)
  {
#ifdef WIFI_SSID
    snprintf(wifiSSID, SSID_SIZE, "%s", WIFI_SSID);
#else
    Serial.println("Introduce wifi SSID:");
    while (ssidSize == 0)
    {
      if (Serial.available())
      {
        ssidSize = serialReceiveBytesUntil(wifiSSID, SSID_SIZE, '\n');
        Serial.printf("SSID: %s, length %i\n", wifiSSID, ssidSize);
        if (ssidSize == 0)
          Serial.println("Please, introduce a valid SSID:");
      }
    }
#endif

#ifdef WIFI_PASSWD
    snprintf(wifiPasswd, PASSWD_SIZE, "%s", WIFI_PASSWD);
#else
    Serial.println("Introduce wifi password (blank if none):");
    while (!passwIntroduced)
    {
      if (Serial.available())
      {
        passwdSize = serialReceiveBytesUntil(wifiPasswd, PASSWD_SIZE, '\n');
        Serial.printf("Password: %s, length %i\n", wifiPasswd, passwdSize);
        passwIntroduced = true;
      }
    }
#endif
    wifiMulti.addAP(wifiSSID, wifiPasswd);

    Serial.printf("Trying to connect to AP with SSID: %s\n", wifiSSID);

    uint8_t status = wifiMulti.run();
    while (status != WL_CONNECTED && status != WL_CONNECT_FAILED && status != WL_NO_SSID_AVAIL)
    {
      status = wifiMulti.run();
      delay(100);
    }
    if (status == WL_CONNECTED)
    {
      Serial.printf("Connected to AP with IP address: %s\n", WiFi.localIP().toString());
      connected = true;
    }
    else
    {
      Serial.printf("There was an error while trying to connect to AP");
      passwIntroduced = false;
    }
  }

  buffer = (char *)malloc(MAX_PACKET_SIZE);
  snprintf(serverAddr, SERVER_ADDR_SIZE, "%s", DEFAULT_SERVER_ADDR);
  serverPort = DEFAULT_SERVER_PORT;

  delay(1000);
  state = HANDSHAKE;
}

ssize_t serialReceiveBytesUntil(char *buffer, const ssize_t size, const char until)
{
  uint8_t i = 0;
  bool terminate = false;
  char aux;

  while (i < size && !terminate)
  {
    if (Serial.available())
    {
      aux = Serial.read();

      if (aux != until && aux != '\r')
      {
        buffer[i] = aux;
        i++;
      }
      else if (aux != '\r')
      {
        buffer[i] = 0;
        terminate = true;
      }
    }
  }

  return i;
}

uint32_t clientReceiveVarInt()
{
  uint32_t ret = 0; /** Return value */
  unsigned char byte;
  unsigned int i = 0;

  /** Introduce the corresponding bit in the final number */

  while (1)
  {
    byte = client.read();
    ret |= (byte & 0x7F) << (7 * i);

    if ((byte & 0x80) != 0x80)
      break;
    i++;
  }

  return ret;
}

ssize_t writeIntToVarInt(char **buffer, unsigned int number)
{
  unsigned int byte;
  ssize_t size = 0;

  while (1)
  {
    byte = number & 0x7F;
    number = number >> 7;

    if (number)
    {
      byte |= 0x80;
      buffer[0][0] = byte;
    }
    else
    {
      buffer[0][0] = byte;
      buffer[0]++;
      size++;
      break;
    }

    buffer[0]++;
    size++;
  }

  return size;
}

void serializeUShort(char **buffer, unsigned short num)
{
  buffer[0][0] = num >> 8;
  buffer[0][1] = num & 0x00ff;

  *buffer += 2;
}

ssize_t sendHandshake(char *buffer)
{
  unsigned int protocolVersion = 774; // Doesn`t really matter, 774 is for minecraft 1.21.11
  char *auxPtr = buffer;
  ssize_t ipLength = strlen(serverAddr) + 1;

  Serial.printf("Building handshake packet...\n");
  /**
   * Meaning of each number (from left to right):
   *  Length of packet ID (1 byte)
   *  Length of protocolVersion number turned into a varInt
   *  Length of server IP
   *  Length of a unsigned short (Server port)
   *  Length of status (1 byte)
   */
  ssize_t dataLength = 1 + writeIntToVarInt(&auxPtr, protocolVersion) + ipLength + sizeof(unsigned short) + 1;

  Serial.printf("[DEBUG HANDSHAKE] DataLength = %i\n", dataLength);

  auxPtr = buffer;

  writeIntToVarInt(&auxPtr, dataLength);
  Serial.printf("[DEBUG HANDSHAKE] DataLength written\n");
  auxPtr[0] = 0x0;
  auxPtr++;
  writeIntToVarInt(&auxPtr, protocolVersion);
  Serial.printf("[DEBUG HANDSHAKE] Protocol version written\n");
  writeIntToVarInt(&auxPtr, ipLength - 1);
  Serial.printf("[DEBUG HANDSHAKE] Server IP length written\n");
  auxPtr += snprintf(auxPtr, ipLength, "%s", serverAddr);
  Serial.printf("[DEBUG HANDSHAKE] Server IP written\n");
  // snprintf(auxPtr, sizeof(unsigned short), "%i", serverPort);
  serializeUShort(&auxPtr, serverPort);
  Serial.printf("[DEBUG HANDSHAKE] Server port written\n");
  auxPtr[0] = 0x1;
  auxPtr++;
  /*auxPtr[0] = '\0';
  auxPtr++;*/

  Serial.printf("[DEBUG HANDSHAKE] Packet buffer: %s\n", buffer);
  state = SERVER_STATUS;
  client.write(buffer, auxPtr - buffer);
  Serial.printf("Handshake sent, packet length = %i\n", auxPtr - buffer);
  return auxPtr - buffer;
}

ssize_t sendStatusRequest(char *buffer)
{
  /* No extra fields are written for this one, apart from the two always required */
  buffer[0] = 0x1; // Data length
  buffer[1] = 0x0; // Packet id

  client.write(buffer, 2);
  Serial.printf("Status request sent, packet length = %i\n", 2);
  return 2;
}

ssize_t receiveServerStatus(struct status_t &status)
{
  JsonDocument doc;
  unsigned int packetLength = clientReceiveVarInt();
  unsigned int packetID = clientReceiveVarInt();
  unsigned int jsonSize = clientReceiveVarInt();
  ssize_t bytesRead = -1;

  Serial.printf("[DEBUG STATUS RESPONSE] Received response, packet length = %i\n", packetLength);
  if (packetLength < MAX_PACKET_SIZE)
  {
    Serial.printf("[DEBUG STATUS RESPONSE] JSON size: %i\n", jsonSize);
    bytesRead = client.readBytes(buffer, jsonSize);

    deserializeJson(doc, buffer);

    // char debugBuffer[MAX_PACKET_SIZE];
    // serializeJsonPretty(doc, debugBuffer);
    // Serial.printf("[DEBUG STATUS RESPONSE] Received data: %s\n", debugBuffer); // The arduinoJson lib can't serialize the version name correctly, it should be serialized as a string

    snprintf(status.versionName, 32, "%s", doc["version"]["name"].as<const char *>());

    Serial.printf("[DEBUG STATUS RESPONSE] Version name: %s\n", doc["version"]["name"].as<const char *>());

    status.onlinePlayers = doc["players"]["online"].as<int>();
    status.maxPlayers = doc["players"]["max"].as<int>();

    state = PING;

    Serial.printf("Status response received!\n");
  }
  else
    Serial.printf("[ERROR] receiveServerStatus error: Not enough buffer size for status response, packet length = %i\n", packetLength);

  client.clear();
  return bytesRead;
}

ssize_t sendPing(char *buffer, long long check)
{
  const unsigned char packetID = 0x01;                                // Packet id
  const unsigned int packetLength = sizeof(packetID) + sizeof(check); // Data length
  char *auxPtr = buffer;

  auxPtr[0] = packetLength;
  auxPtr++;
  auxPtr[0] = packetID;
  auxPtr++;
  auxPtr += snprintf(auxPtr, sizeof(check) + 1, "%lli", check);

  client.write(buffer, auxPtr - buffer);
  Serial.printf("Ping sent, packet length = %i\n", auxPtr - buffer);
  client.clear();
  return auxPtr - buffer;
}

ssize_t receivePong(char *buffer, struct status_t &status, unsigned long timer, long long check)
{
  unsigned int packetLength = clientReceiveVarInt();
  unsigned int packetID = clientReceiveVarInt();
  ssize_t bytesRead = -1;

  if (packetLength < MAX_PACKET_SIZE) // TODO: Provisional margin, change
  {
    bytesRead = client.readBytes(buffer, packetLength - 1); // packetLength - packetID
    buffer[packetLength - 1] = 0;
    printf("[DEBUG PONG] Received pong: %s\n", buffer);

    if (atoll(buffer) == check)
      status.ping = millis() - timer;
    else
    {
      printf("[DEBUG PONG] Received pong number doesn't match the one sent\n");
      status.ping = -1;
    }
  }
  else
  {
    Serial.printf("[ERROR] receivePong error: Not enough buffer size for pong response, packet length = %i\n", packetLength);
    status.ping = -1;
  }
  client.clear();
  return bytesRead;
}

void showServerStatus(const struct status_t &status)
{
  String players = String("Players: " + String(status.onlinePlayers) + "/" + String(status.maxPlayers));
  String version = String("Version: " + String(status.versionName));
  String ping = String("Ping: " + String(status.ping) + "ms");
  uint8_t size = 1;
  int16_t x = 1;

  Serial.printf("[SERVER STATUS] version: %s, online: %i, max: %i\n", status.versionName, status.onlinePlayers, status.maxPlayers);

  display.clearDisplay();
  displayTitle();
  printOnDisplay(size, SSD1306_WHITE, x, 20, players);
  if (!textOverflows(size, version, x))
    printOnDisplay(size, SSD1306_WHITE, x, 30, version);
  else
  {
    version = String("V: " + String(status.versionName));
    if (!textOverflows(size, version, x))
      printOnDisplay(size, SSD1306_WHITE, x, 30, version);
  }
  printOnDisplay(size, SSD1306_WHITE, x, 40, ping);
}

void printOnDisplay(uint8_t size, uint16_t color, int16_t x, int16_t y, String text, bool cp437)
{
  display.cp437(cp437);
  display.setTextSize(size);
  display.setTextColor(color);
  display.setCursor(x, y);
  if (text)
    display.printf(text.c_str());
  else
    Serial.printf("[ERROR] printOnDisplay error: Invalid string for text\n");
  display.display();
}

void displayTitle()
{
  uint8_t size = 1;
  String title = String("Status of " + String(serverAddr));
  int16_t x = centerXText(size, title);
  if (title.length() * BASE_PIXELS_PER_CHAR > SCREEN_WIDTH - x)
  {
    title = String(serverAddr);
    x = centerXText(size, title);
  }
  printOnDisplay(size, SSD1306_WHITE, x, 0, title);
}

bool textOverflows(int sizeFactor, String text, uint16_t x)
{
  return SCREEN_WIDTH <= (x + text.length() * sizeFactor * BASE_PIXELS_PER_CHAR);
}

int16_t centerXText(int sizeFactor, String text)
{
  return (SCREEN_WIDTH - (text.length() * sizeFactor * BASE_PIXELS_PER_CHAR)) / 2;
}

void loop()
{
  struct status_t status;
  int timer;
  long long pingCheck = 12345678;

  if (Serial.available())
  {
    char command[10];
    char args[115];
    serialReceiveBytesUntil(buffer, 128, '\n');
    if (sscanf(buffer, "%s %s", command, args) == 2)
    {
      if (strcmp(command, "a") == 0)
      {
        snprintf(serverAddr, SERVER_ADDR_SIZE, "%s", &buffer[2]);
        Serial.printf("Changed target server address to %s\n", serverAddr);
      }
      else if (strcmp(command, "p") == 0)
      {
        serverPort = atoi(&buffer[2]);
        Serial.printf("Changed target server port to %hu\n", serverPort);
      }
      else
        Serial.printf("Unrecognised command: %s\n", command);
    }
    else
      Serial.printf("Invalid command format: %s\n", buffer);
  }
  if (client.connect(serverAddr, serverPort))
  {
    state = HANDSHAKE;

    Serial.printf("Connected to server %s:%hu!\n", serverAddr, serverPort);

    sendHandshake(buffer);

    /* Server doesn`t answer to the handshake*/
    sendStatusRequest(buffer);

    while (!client.available())
      ;
    receiveServerStatus(status);

    timer = millis();
    sendPing(buffer, pingCheck);

    while (!client.available())
      ;
    receivePong(buffer, status, timer, pingCheck);

    showServerStatus(status);

    client.stop();
  }
  else
  {
    Serial.printf("Could not connect to the minecraft server in address %s:%hu\n", serverAddr, serverPort);
    display.clearDisplay();
    displayTitle();
    /**
     *  5*size pixels per character
     *
     *  x = (SCREEN_WIDTH - (word letters * size * BASE_PIXELS_PER_CHAR)) / 2
     *
     */
    int size = 2;
    String conn = String("Connection");
    String err = String("error");
    printOnDisplay(size, SSD1306_WHITE, centerXText(size, conn), 20, conn);
    printOnDisplay(size, SSD1306_WHITE, centerXText(size, err), 40, err);
  }
  delay(10000);
}
