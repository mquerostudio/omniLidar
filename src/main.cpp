#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>

#include <SimpleFOC.h>
#include <VL53L1X.h>

#include <math.h>
#include <vector>
#include "main.h"

WebServer server(80);
WebSocketsServer webSocket(81); // create a websocket server on port 81
/*********************************************/

/*
 * Variables Declarations
 */
int lidarMaxDist = DEFLIDARMAXDIST, lidarMeasperRev = DEFLIDARMEASPERREV, lidarTimeofRev = DEFLIDARTIMEOFREV, lidarMeasTime = DEFLIDARMEASTIME;
float lidarOffsetbwMeas = (2 * PI) / lidarMeasperRev;
float lidarTimeofMeas = lidarTimeofRev / lidarMeasperRev;

float motorPosition = INITIALMOTPOST;

// Vector arrays to store data
std::vector<std::vector<float>> coordsPolar;
std::vector<std::vector<float>> coordsCartesian;

// Variables to store the value from webserver
int toPutLidarMaxDist, toPutlidarMeasperRev, toPutlidarTimeofRev, toPutlidarMeasTime;
bool isPutNewValues = false, isStarted = false;

enum_state state = MOTORHOMING;

uint16_t niteration = 0;
unsigned long previousUpdateTime = 0;
/*********************************************/

/*  I2C Objects Declaration  */
TwoWire I2CZero = TwoWire(0);
TwoWire I2COne = TwoWire(1);

/*  VL53L1X Object Declaration  */
VL53L1X VL53L1Xsensor;

/*  Motor Objects Declarations */
MagneticSensorI2C AS5600sensor = MagneticSensorI2C(AS5600_I2C); // Encoder of the motor
BLDCMotor motor = BLDCMotor(7);                                 // Number of poles
BLDCDriver3PWM motorDriver = BLDCDriver3PWM(26, 27, 14, 12);

void handleRoot()
{
  server.send(200, "text/html", htmlContent);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[%u] Disconnected!\n", num);
    break;
  case WStype_CONNECTED:
  {
    IPAddress ip = webSocket.remoteIP(num);
    Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
  }
  break;
  case WStype_TEXT:

    if (length > 0 && payload[0] == '{')
    { // Check if payload is a JSON string (basic check)
      // Parse JSON payload for form data
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, (char *)payload);
      toPutlidarMeasTime = doc["lidarTime"];
      toPutlidarTimeofRev = doc["measTimePerRev"];
      toPutLidarMaxDist = doc["lidarMaxDistance"];
      toPutlidarMeasperRev = doc["measNumPerRev"];
      // Print the values (you can remove this if not needed)
      Serial.println("Values saved via WebSocket:");
      Serial.print("Lidar Time: ");
      Serial.println(toPutlidarMeasTime);
      Serial.print("Meas Time per Revolution: ");
      Serial.println(toPutlidarTimeofRev);
      Serial.print("Lidar Max Meas Distance: ");
      Serial.println(toPutLidarMaxDist);
      Serial.print("Meas Num per Revolution: ");
      Serial.println(toPutlidarMeasperRev);

      // Send updated lidarMaxDistance to frontend
      String lidarMaxDistanceUpdate = "lidarMaxDistanceUpdate:" + String(toPutLidarMaxDist);
      webSocket.broadcastTXT(lidarMaxDistanceUpdate.c_str());
      return;
    }

    String msg = String((char *)payload);
    if (msg == "start")
    {
      Serial.println("Start Button");
      isStarted = true;
      webSocket.broadcastTXT("isStarted");
    }
    else if (msg == "stop")
    {
      Serial.println("Stop Button");
      isStarted = false;
      webSocket.broadcastTXT("isStopped");
    }
    break;
  }
}

void setup()
{
  /*
   * Serial Init and configuration
   */
  Serial.begin(SERIALVEL);
  delay(1000);
  Serial.print(F("Serial Velocity: "));
  Serial.println(SERIALVEL);
  /*********************************************/

  /*
   * WiFi Setup
   */
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP Address ");
  Serial.println(WiFi.localIP());

  //  Use mdns for host name resolution
  if (!MDNS.begin(host)) // http://omniLidar.local
  {
    Serial.println("Error setting up MDNS responder!");
    delay(5000);
    Serial.println("mDNS responder started");
  }

  server.on("/", handleRoot); // Start Main Page

  // OTA Conf
  server.on("/serverIndex", HTTP_GET, []()
            {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex); });

  /*handling uploading firmware file */
  server.on(
      "/update", HTTP_POST, []()
      {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); },
      []()
      {
        HTTPUpload &upload = server.upload();
        if (upload.status == UPLOAD_FILE_START)
        {
          Serial.printf("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN))
          { // start with max available size
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_WRITE)
        {
          /* flashing firmware to ESP*/
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
          {
            Update.printError(Serial);
          }
        }
        else if (upload.status == UPLOAD_FILE_END)
        {
          if (Update.end(true))
          { // true to set the size to the current progress
            Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
          }
          else
          {
            Update.printError(Serial);
          }
        }
      });

  server.begin();
  Serial.println("HTTP server started");

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");
  /*********************************************/

  /*
   * I2C Setups
   */
  if (!I2CZero.begin(SDA0, SCL0, I2CSPEED))
  {
    Serial.println(F("I2C 0 - NOT initiated."));
  }
  Serial.println(F("I2C 0 - successfully initiated."));

  if (!I2COne.begin(SDA1, SCL1, I2CSPEED))
  {
    Serial.println(F("I2C 1 - NOT initiated."));
  }
  Serial.println(F("I2C 1 - successfully initiated."));
  /*********************************************/

  /*
   * Motor Setups
   */
  AS5600sensor.init(&I2COne);
  Serial.println(F("Motor - Encoder initiated."));
  motor.linkSensor(&AS5600sensor);
  Serial.println(F("Motor - Encoder linked to motor."));

  // Motor driver conf
  motorDriver.voltage_power_supply = PSVOLTAGE;
  Serial.print(F("Motor - Power supply voltage: "));
  Serial.println(PSVOLTAGE);
  motorDriver.init();

  // Motor conf
  motor.linkDriver(&motorDriver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  Serial.println(F("Motor - Modulation Type: Space Vector PWM."));
  motor.controller = MotionControlType::angle;
  Serial.println(F("Motor - Control Type: angle/position."));
  motor.PID_velocity.P = PIDVELP;
  motor.PID_velocity.I = PIDVELI;
  motor.P_angle.P = PIDPOSP;
  motor.LPF_velocity.Tf = TFLPFVEL;
  Serial.println(F("Motor - PID Values:"));
  Serial.print(F("Motor - Velocity P: "));
  Serial.println(PIDVELP);
  Serial.print(F("Motor - Velocity I: "));
  Serial.println(PIDVELI);
  Serial.print(F("Motor - Position P: "));
  Serial.println(PIDPOSP);
  Serial.print(F("Motor - Time constant LPF: "));
  Serial.println(TFLPFVEL);
  motor.voltage_limit = MOTORVOLLIMIT;
  Serial.print(F("Motor - Voltage Limit: "));
  Serial.println(MOTORVOLLIMIT);
  motor.velocity_limit = MOTORVELLIMIT;
  Serial.print(F("Motor - Velocity Limit: "));
  Serial.println(MOTORVELLIMIT);
  motor.useMonitoring(Serial);

  // Init Motor and FOC
  motor.init();
  motor.initFOC();
  /*********************************************/

  /*
   * VL53L1X Setup
   */
  // Initialize the xshut pin and power on the VL53L1X
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);
  delay(10); // Wait for a short duration before enabling the sensor
  digitalWrite(XSHUT_PIN, HIGH);
  delay(10); // Wait for sensor to power up

  VL53L1Xsensor.setBus(&I2CZero);
  // Initialize the VL53L1X sensor
  if (!VL53L1Xsensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1)
      ;
  }
  // Start continuous measurements
  VL53L1Xsensor.startContinuous(lidarMeasTime);

  /*
   * Init main program
   */
  Serial.println(F("State " + state));

  delay(60000);
}

void loop()
{
  server.handleClient();
  ArduinoOTA.handle();
  webSocket.loop();
  motor.loopFOC();
  motor.move(motorPosition);

  switch (state)
  {
  case MOTORHOMING:
    motorHoming();
    break;
  case CHANGESETTINGS:
    changeSettings();
    break;
  case MEASDATA:
    normalOperation();
    break;
  case SENDDATA:
    sendData();
    break;
  default:
    break;
  }
}

void motorHoming()
{
  motorPosition = INITIALMOTPOST;
  if (AS5600sensor.getAngle() < (INITIALMOTPOST + 0.02) || AS5600sensor.getAngle() < fabs(INITIALMOTPOST - 0.02))
  {
    Serial.println(F("Motor - Home Position"));
    state = CHANGESETTINGS;
    Serial.println(F("State " + state));
  }
}

void changeSettings()
{
  if (isStarted)
  {
    state = MEASDATA;
    Serial.println(F("State " + state));
  }
}

void normalOperation()
{
  if ((millis() - previousUpdateTime) > lidarTimeofMeas)
  {
    previousUpdateTime = millis();
    motorPosition += lidarOffsetbwMeas;

    Serial.print(F("Numero de iteracion: "));
    Serial.println(niteration);

    uint16_t distance = VL53L1Xsensor.read();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" mm");

    coordsPolar[niteration][0] = distance;
    coordsPolar[niteration][1] = AS5600sensor.getMechanicalAngle();

    if (niteration == (lidarMeasperRev - 1))
    {
      Serial.println(F("Data - Data ready"));
      state = SENDDATA;
      Serial.println(F("State " + state));
      niteration = 0;
    }
    else
    {
      niteration += 1;
    }
  }
}

void sendData()
{
  String dataToSend = "";

  for (int i = 0; i < lidarMeasperRev; i++)
  {
    dataToSend += String(coordsPolar[i][0]) + "," + String(coordsPolar[i][1]);
    if (i > 0)
      dataToSend += "|";
  }
  if (!webSocket.broadcastTXT(dataToSend))
  {
    Serial.println("Error Sending Data to web");
  }
  else if (isStarted)
  {
    state = MEASDATA;
    Serial.println(F("State " + state));
  } else{
    state = CHANGESETTINGS;
  }
}