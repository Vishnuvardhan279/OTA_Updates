#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_ADS1X15.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#define Relaypin 26

#define MDASH_APP_NAME "MinimalApp"
#include <mDash.h>

const char* ssid = "Shree_Hyd";
const char* password = "7995009229";
#define DEVICE_PASSWORD "QLx2mdZhp99mHeJjbm5Rvcw"
const char* serverUrl = "http://34.31.8.26:3002/api/newcellsdata"; // Update with your server's IP and port
const char* relayUrl = "http://34.31.8.26:3002/api/relay";
const char* relaydataUrl = "http://34.31.8.26:3002/api/relaydata";

float cell_1, cell_2, cell_3, cell_4;
double current_1, current_2, current_3, current_4;
float thresholdvalue=0.1;  // set the threshold value for difference between previus value and present value to send the data
float tempC;
float R1 = 30000.0;
float R2 = 7500.0;
const float batteryCapacity_mAh = 6000.0;  //mAh
float SoC_1 = 100.0; // Start with full charge
float SoC_2 = 100.0;
float SoC_3 = 100.0;
float SoC_4 = 100.0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;

WiFiClient client;
Adafruit_ADS1115 ads1; /* Use this for the 16-bit version */
Adafruit_ADS1115 ads2;
OneWire oneWire(4);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
int numberOfDevices;

float lastCell1Value = 0.0;
float lastCell2Value = 0.0;
float lastCell3Value = 0.0;
float lastCell4Value = 0.0;
double lastCurrent1Value = 0.0;
float lastTempC = 0.0;


void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      Serial.print(deviceAddress[i], HEX);
  }
}
float voltageToSoC(float voltage) {
    // Replace with actual voltage-SOC relationship for LFP cells
    if (voltage >= 3.65) return 100;
    if (voltage <= 2.50) return 0;
    // Linear interpolation as an example
    return (voltage - 2.50) / (3.65 - 2.50) * 100;
}
void setup() {
  pinMode(Relaypin, OUTPUT);
  Serial.begin(115200);
  delay(1000);
previousTime = millis();
  // Connect to WiFi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");
  mDashBegin(DEVICE_PASSWORD);

  ads1.begin(0x48);
  ads2.begin(0x49);
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
   Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" temperature devices.");
   for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }
  if (!ads1.begin(0x48))
  {
    Serial.println("Failed to initialize ADS1.");
    while (1);
  }
   if (!ads2.begin(0x49))
  {
    Serial.println("Failed to initialize ADS2.");
    while (1);
  }
    lastCell1Value = cell_1;
  lastCell2Value = cell_2;
  lastCell3Value = cell_3;
  lastCell4Value = cell_4;
  lastCurrent1Value = current_1;
  lastTempC = tempC;
}


bool fetchRelayState() {
  HTTPClient http;
  http.begin(client, relayUrl);
  int httpResponseCode = http.GET();
  bool relayState = false;

  if (httpResponseCode > 0) {
    String payload = http.getString();
    Serial.println("Relay state response: " + payload);

    // Allocate the JSON document
    StaticJsonDocument<200> doc;

    // Deserialize the JSON document
    DeserializationError error = deserializeJson(doc, payload);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return false;
    }

    // Extract the state
    const char* state = doc["state"];
    Serial.println("Extracted state: " + String(state));
    relayState = (String(state) == "ON");
    Serial.println("Relay state set to: " + String(relayState));
  } else {
    Serial.println("Error on relay state GET request");
  }

  http.end();
  return relayState;
}


void sendRelayStatus(bool relayStatus) {
  // Create JSON data
  String relayStatusString = relayStatus ? "ON" : "OFF";
  String jsonData1 = "{\"relay_status\":\"" + relayStatusString + "\"}";

  // Send POST request to server
  HTTPClient http;
  http.begin(client, relaydataUrl);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode1 = http.POST(jsonData1);

  if (httpResponseCode1 > 0) {
    String response1 = http.getString();
    Serial.println("Relay status response: " + response1);
  } else {
    Serial.println("Error sending relay status POST request: " + String(httpResponseCode1));
  }
  http.end();
}

void loop() {
  currentTime = millis();
   float elapsedTime = (currentTime - previousTime) / 3600000.0; // Time in hours
    previousTime = currentTime;
  bool relayState = fetchRelayState();
  //Serial.println(relayState);
  digitalWrite(Relaypin, relayState ? HIGH : LOW);
  // Read sensor data
  int16_t adc1_0, adc1_1, adc1_2, adc1_3;
  int16_t adc2_0, adc2_1, adc2_2, adc2_3;

  adc1_0 = ads1.readADC_SingleEnded(0);
  adc1_1 = ads1.readADC_SingleEnded(1);
  adc1_2 = ads1.readADC_SingleEnded(2);
  adc1_3 = ads1.readADC_SingleEnded(3);

  adc2_0 = ads2.readADC_SingleEnded(0);
  adc2_1 = ads2.readADC_SingleEnded(1);
  adc2_2 = ads2.readADC_SingleEnded(2);
  adc2_3 = ads2.readADC_SingleEnded(3);

  // Calculate voltages and currents
  float volts1_0, volts1_1, volts1_2, volts1_3;
  float volts2_0, volts2_1, volts2_2, volts2_3;

  volts1_0 = ads1.computeVolts(adc1_0);     //cell 1
  volts1_1 = ads2.computeVolts(adc2_1)-0.02 ;    // cell 2 connected to adc 2
  volts1_2 = ads1.computeVolts(adc1_2)-0.02 ;    //cell 3
  volts1_3 = ads1.computeVolts(adc1_3)-0.03 ;    //cell 4

  volts2_0 = ads2.computeVolts(adc2_0);     // current 1
  //volts2_1 = ads2.computeVolts(adc2_1);     
  volts2_2 = ads2.computeVolts(adc2_2);
  volts2_3 = ads2.computeVolts(adc2_3);
  
  float Voltage_1 = volts1_0 * 5.00;
  float Voltage_2 = volts1_1 * 5.05;
  float Voltage_3 = volts1_2 * 5.00;
  float Voltage_4 = volts1_3 * 5.02;

  cell_1 = Voltage_1;  // V1
  cell_2 = Voltage_2 - Voltage_1;  // V2
  cell_3 = Voltage_3 - Voltage_2;  // V3
  cell_4 = Voltage_4 - Voltage_3;  // V4

  current_1 = (volts2_0 - 2.50) / 0.066;  // C1
  current_2 = (volts2_1 - 2.5) / 0.066;  // C2
  current_3 = (volts2_2 - 2.5) / 0.066;  // C3
  current_4 = (volts2_2 - 2.5) / 0.066;  // C4
  float tempC;

  // Read temperature
  sensors.requestTemperatures(); for(int i=0;i<numberOfDevices; i++){
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)){
      // Output the device ID
      Serial.print("Temperature: ");
     // Serial.print(i,DEC);
      // Print the data
      tempC = sensors.getTempC(tempDeviceAddress); 
    //  Serial.print("Temp C: ");
      Serial.println(tempC);        //Temperature
    }
  }
  float chargeChange = current_1 * elapsedTime; // Charge change in Ah
    SoC_1 -= (chargeChange / batteryCapacity_mAh) * 100;
    SoC_2 -= (chargeChange / batteryCapacity_mAh) * 100;
    SoC_3 -= (chargeChange / batteryCapacity_mAh) * 100;
    SoC_4 -= (chargeChange / batteryCapacity_mAh) * 100;
     if (SoC_1 > 100) SoC_1 = 100;
    if (SoC_1 < 0) SoC_1 = 0;
    if (SoC_2 > 100) SoC_2 = 100;
    if (SoC_2 < 0) SoC_2 = 0;
    if (SoC_3 > 100) SoC_3 = 100;
    if (SoC_3 < 0) SoC_3 = 0;
    if (SoC_4 > 100) SoC_4 = 100;
    if (SoC_4 < 0) SoC_4 = 0;
if(current_1<-5)
{
  current_1=0;
}
    bool relayStatus = digitalRead(Relaypin);
  sendRelayStatus(relayStatus);
  // Create JSON payload
  if (abs(cell_1 - lastCell1Value) >= thresholdvalue || abs(cell_2 - lastCell2Value) >= thresholdvalue || abs(cell_3 - lastCell3Value) >= thresholdvalue || abs(cell_4 - lastCell4Value) >= thresholdvalue || abs(current_1 - lastCurrent1Value) >= 0.3 || abs(tempC - lastTempC) >= 2.0) {
String jsonData = "{\"cell1_voltage\":" + String(cell_1) + ", \"cell2_voltage\":" + String(cell_2) + ", \"cell3_voltage\":" + String(cell_3) + ", \"cell4_voltage\":" + String(cell_4) + ", \"overall_temperature\":" + String(tempC) + ", \"overall_current\":" + String(current_1) + "}";


  // Send POST request to server
  HTTPClient http;
  http.begin(client, serverUrl);
  http.addHeader("Content-Type", "application/json");
  int httpResponseCode = http.POST(jsonData);

  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Server response: " + response);
    lastCell1Value = cell_1;
    lastCell2Value = cell_2;
    lastCell3Value = cell_3;
    lastCell4Value = cell_4;
    lastCurrent1Value = current_1;
    lastTempC = tempC;
  } else {
    Serial.println("Error sending cell data POST request");
  }

  http.end();
 }

//  if (cell_1 < 2.2 || cell_1 > 3.6 ||
//      cell_2 < 2.2 || cell_2 > 3.6 ||
//      cell_3 < 2.2 || cell_3 > 3.6 ||
//      cell_4 < 2.2 || cell_4 > 3.6 ||tempC > 45.0)
//  {
//    digitalWrite(Relaypin, LOW);  // Turn off relay
//    Serial.println("Relay turned off");
//  } 
//  else
//  {
//    digitalWrite(Relaypin, HIGH);  // Turn on relay
//    Serial.println("Relay turned on");
//  }


//Serial.print("AIN1_0: "); Serial.print(adc1_0); Serial.print("  ");
// Serial.print(volts1_0); Serial.println("V");
// Serial.print(Voltage_1); Serial.println("V");
  Serial.print("cell 1 voltage = "); Serial.print(cell_1); Serial.println("V");
// Serial.print("AIN1_1: "); Serial.print(adc1_1); Serial.print("  "); 
//Serial.print(volts1_1); Serial.println("V");
//  Serial.print(Voltage_2); Serial.println("V");
  Serial.print("cell 2 voltage = "); Serial.print(cell_2); Serial.println("V");
//   Serial.print("AIN1_2: "); Serial.print(adc1_2); Serial.print("  "); 
//  Serial.print(volts1_2); Serial.println("V");
//Serial.print(Voltage_3); Serial.println("V"); 
  Serial.print("cell 3 voltage = ");  Serial.print(cell_3); Serial.println("V");
//  Serial.print("AIN1_3: "); Serial.print(adc1_3); Serial.print("  "); 
//  Serial.print(volts1_3); Serial.println("V");
//  Serial.print(Voltage_4); Serial.println("V");
  Serial.print("cell 4 voltage = ");  Serial.print(cell_4); Serial.println("V");
  //Serial.print("AIN2_0: "); Serial.print(adc2_0); Serial.print("  "); 
  //Serial.print(volts2_0); Serial.println("V");
Serial.print("battery pack current="); Serial.print(current_1); Serial.println("A");
//  Serial.print("AIN2_1: "); Serial.print(adc2_1); Serial.print("  "); 
//  Serial.print(volts2_1); Serial.println("V");
//Serial.print("cell 2 current="); Serial.print(current_2); Serial.println("A");
//  Serial.print("AIN2_2: "); Serial.print(adc2_2); Serial.print("  "); 
//  Serial.print(volts2_2); Serial.println("V");
//Serial.print("cell 3 current="); Serial.print(current_3); Serial.println("A");
//  Serial.print("AIN2_3: "); Serial.print(adc2_3); Serial.print("  "); 
//  Serial.print(volts2_3); Serial.println("V");
//Serial.print("cell 4 current="); Serial.print(current_4); Serial.println("A");
Serial.print("SoC_1: "); Serial.println(SoC_1);
    Serial.print("SoC_2: "); Serial.println(SoC_2);
    Serial.print("SoC_3: "); Serial.println(SoC_3);
    Serial.print("SoC_4: "); Serial.println(SoC_4);
Serial.println("-----------------------------------------------------------");
  delay(60000);  // checks data for every minute
}
