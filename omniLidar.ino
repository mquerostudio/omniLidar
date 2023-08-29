#include <SimpleFOC.h>
#include <Adafruit_VL53L1X.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#define SERIALVEL 115200

#define SDA0 19
#define SCL0 18
#define SDA1 23
#define SCL1 5
#define i2cspeed 400000u

#define IRQ_PIN 15
#define XSHUT_PIN 13
/**/#define TIMINGLIDAR 20  // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
/**/#define MAXDISTANCE 2000

#define PSVOLTAGE 12  //Voltage nominal
#define PIDVELP 0.05
#define PIDVELI 1
#define PIDPOSP 20
#define TFLPFVEL 0.01
#define MOTORVOLLIMIT 17
#define MOTORVELLIMIT 20
#define INITIALMOTPOST 0.0

/**/#define NUMMEASPERREV 4
/**/#define OFFSET ((2 * PI) / NUMMEASPERREV)
/**/#define TIMEOFREV 2000  // millisSEG
/**/#define TIMEBEWOFF (TIMEOFREV / NUMMEASPERREV)

#define CANVASWIDTH 900
#define CANVASHEIGHT CANVASWIDTH
#define CANVASNORMALIZE (CANVASWIDTH / (2 * MAXDISTANCE))


TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(26, 27, 14, 12);
WebServer server(80);

int16_t distance;
float motorPos;

float arrayDistancesMod[NUMMEASPERREV][2];  // [Distance in mm] [Angle in rad]
float arrayDistancesXY[NUMMEASPERREV][2];   // [x in mm] [y in mm]

uint16_t niteration = 0;
unsigned long previousUpdateTime = 0;

const char* ssid = "TP-LINK_26619E";
const char* password = "18670691";

uint8_t state = 0;
/*
* 0 --- Wifi Setup
* 1 --- First Run - Motor Homing
* 2 --- Normal Operation
* 3 --- Send Data
*/

void initWifi();
void firstRun();
void normalOperation();
void sendData();


String generateHTML() {
  String html = "<html><head><style>body{font-family: Arial, Helvetica, sans-serif;}</style></head><body>";
  html += "<h1>Lidar Measurement</h1>";
  html += "<canvas id='graphCanvas' width='" + String(CANVASWIDTH) + "' height='" + String(CANVASWIDTH) + "'></canvas>";
  html += "<script>";
  html += "var canvas = document.getElementById('graphCanvas');";
  html += "var ctx = canvas.getContext('2d');";
  html += "ctx.fillStyle = 'lightgray';";
  html += "ctx.fillRect(0, 0, canvas.width, canvas.height);";
  html += "ctx.fillStyle = 'blue';";

  html += "ctx.strokeStyle = 'black';";
  html += "ctx.beginPath();";
  html += "ctx.moveTo(canvas.width / 2, 0);";
  html += "ctx.lineTo(canvas.width / 2, canvas.height);";
  html += "ctx.stroke();";

  html += "ctx.beginPath();";
  html += "ctx.moveTo(0, canvas.height / 2);";
  html += "ctx.lineTo(canvas.width, canvas.height / 2);";
  html += "ctx.stroke();";

  for (int i = 0; i < NUMMEASPERREV; i++) {
    float x = CANVASNORMALIZE * (arrayDistancesXY[i][0] + CANVASWIDTHHEIGHT / 2);
    if (x > CANVASWIDTHHEIGHT) {
      x = CANVASWIDTHHEIGHT;
    }
    float y = CANVASNORMALIZE * (arrayDistancesXY[i][1] + CANVASWIDTHHEIGHT / 2);
    if (y > CANVASWIDTHHEIGHT) {
      y = CANVASWIDTHHEIGHT;
    }
    html += "ctx.fillRect(" + String(x) + ", " + String(y) + ", 10, 10);";
  }

  html += "</script>";
  // Auto-refresh the page every updateInterval milliseconds
  html += "<script>setTimeout(function() { location.reload(); }, " + String(TIMEOFREV) + ");</script>";
  html += "</body></html>";
  return html;
}

void handleRoot() {
  server.send(200, "text/html", generateHTML());
}

void setup() {

  Serial.begin(SERIALVEL);
  delay(100);
  Serial.print(F("Serial Velocity: "));
  Serial.println(SERIALVEL);

  // i2c setup
  if (!I2Cone.begin(SDA0, SCL0, i2cspeed)) {
    Serial.println(F("I2C 0 - NOT initiated."));
  }
  Serial.println(F("I2C 0 - successfully initiated."));

  if (!I2Ctwo.begin(SDA1, SCL1, i2cspeed)) {
    Serial.println(F("I2C 1 - NOT initiated."));
  }
  Serial.println(F("I2C 1 - successfully initiated."));

  as5600.init(&I2Ctwo);
  Serial.println(F("Motor - Encoder initiated."));

  motor.linkSensor(&as5600);
  Serial.println(F("Motor - Encoder linked to motor."));

  // Motor driver conf
  driver.voltage_power_supply = PSVOLTAGE;
  Serial.print(F("Motor - Power supply voltage: "));
  Serial.println(PSVOLTAGE);
  driver.init();

  // Motor conf
  motor.linkDriver(&driver);
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


  // Lidar Init
  if (!vl53.begin(0x29, &I2Cone)) {
    Serial.print(F("Lidar - Error on init Lidar sensor: "));
    Serial.println(vl53.vl_status);
  }
  Serial.println(F("Lidar - VL53L1X sensor OK"));

  Serial.print("Lidar - Set timing budget (ms): ");
  Serial.println(TIMINGLIDAR);
  vl53.setTimingBudget(TIMINGLIDAR);
  Serial.print(F("Lidar - Get timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());

  if (!vl53.startRanging()) {
    Serial.print(F("Lidar - Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
  }
  Serial.println(F("Lidar - Ranging started"));

  motorPos = INITIALMOTPOST;
}

void loop() {
  motor.loopFOC();
  motor.move(-(motorPos));
  server.handleClient();

  switch (state) {
    case 0:
      initWifi();
      break;
    case 1:
      firstRun();
      break;

    case 2:
      normalOperation();
      break;

    case 3:
      sendData();
      break;

    case 4:
      break;
  }
}

void initWifi() {

  Serial.println(F("State 0 - Starting WiFi"));
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println(F("WiFi - Conectado"));

  server.on("/", handleRoot);
  server.begin();
  Serial.println(F("WiFi - Servidor Iniciado"));


  Serial.println(F("State 1 - First Run Detected"));
  Serial.println(F("Motor - Homing"));
  state = 1;
}

void firstRun() {
  float angpost = as5600.getMechanicalAngle();
  // Serial.print(F("Encoder - Posicion Actual: "));
  // Serial.println(angpost);

  if (angpost < (INITIALMOTPOST + 0.01)) {
    Serial.println(F("Motor - Home Position"));
    state = 2;
    Serial.println(F("State 2 - Normal Operation"));
  }
}

void normalOperation() {

  if ((millis() - previousUpdateTime) > TIMEBEWOFF) {
    previousUpdateTime = millis();


    Serial.print(F("Numero de iteracion: "));
    Serial.println(niteration);

    motorPos = motorPos + OFFSET;

    // Start measurement
    if (vl53.dataReady()) {
      distance = vl53.distance();
      if (distance == -1) {
        // something went wrong!
        Serial.print(F("Lidar - Couldn't get distance: "));
        Serial.println(vl53.vl_status);
      }
    }

    // data is read out, time for another reading!
    vl53.clearInterrupt();

    Serial.print(as5600.getMechanicalAngle());
    Serial.print(F(" - "));
    Serial.print(distance);
    Serial.print(F(" - "));
    Serial.print(TIMEBEWOFF);
    Serial.print(F(" - "));
    Serial.println(OFFSET);

    arrayDistancesMod[niteration][0] = distance;
    arrayDistancesMod[niteration][1] = as5600.getMechanicalAngle();

    if (niteration == (NUMMEASPERREV - 1)) {
      Serial.println(F("Data - Data ready"));
      state = 3;
      niteration = 0;
      Serial.println(F("State 3 - Send Data"));
    } else {
      niteration += 1;
    }
  }

  distance = -2;
}

// // float arrayDistances[NUMMEASPERREV][2];
void sendData() {

  Serial.println(F("Data - Package of Data:"));
  Serial.println(F("[Distance in mm] [Angle in rad]"));

  for (int i = 0; i < NUMMEASPERREV; i++) {
    for (int t = 0; t < 2; t++) {
      Serial.print(F(":"));
      Serial.print(arrayDistancesMod[i][t]);
    }
    Serial.println(F(";"));
  }

  //BORRADO
  for (int i = 0; i < NUMMEASPERREV; i++) {
    for (int t = 0; t < 2; t++) {
      arrayDistancesMod[i][t] = 0;
    }
  }

  Serial.println(F("Data - Data Sent."));
  state = 1;
  Serial.println(F("State 1 - Normal Operation"));
}