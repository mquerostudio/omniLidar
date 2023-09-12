#define SERIALVEL 115200

/* I2C PINS
 * I2C0 --> VL53L1X
 * I2C1 --> AS5600
 */
#define SDA0 19
#define SCL0 18
#define SDA1 23
#define SCL1 5
#define I2CSPEED 400000u

/*
 * VL53L1X Pins
 */
#define IRQ_PIN 15
#define XSHUT_PIN 13

/*
 * MOTOR PARAMETERS
 */
#define PSVOLTAGE 14.8f // nominal voltage of the battery
#define MOTORVOLLIMIT 17
#define MOTORVELLIMIT 20
#define PIDVELP 0.05f
#define PIDVELI 1
#define PIDPOSP 20
#define TFLPFVEL 0.01f
#define INITIALMOTPOST 0.0f

/*
 * Defaults Values
 */
#define DEFLIDARMAXDIST 2000   // Max meas distance of the sensor
#define DEFLIDARMEASPERREV 10  // Number Meas of each revolution
#define DEFLIDARTIMEOFREV 2000 // in ms
#define DEFLIDARMEASTIME 50    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms

/*
 * Web Server Parameters
 */
const char *host = "omnilidar";
const char *ssid = "TP-LINK_26619E";
const char *password = "18670691";

/*
 * 0 --> Motor Homing
 * 1 --> Change Settings
 * 2 --> Meas
 * 3 --> Send Data
 */
enum enum_state
{
  MOTORHOMING,
  CHANGESETTINGS,
  MEASDATA,
  SENDDATA
};

void motorHoming();
void normalOperation();
void sendData();
void stopMotor();
void changeSettings();

const char *htmlContent = R"(
<!DOCTYPE html>
<html lang="es">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>omniLidar Web Control</title>
    <style>
        h1 {
          color: green;
          font-size: 40px;
        }
        body {
            background-color: #FFD700;
            font-family: Arial, sans-serif;
            margin: 0;
            display: flex;
            text-align: center;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            height: 100vh;
        }

        .container {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 10px;
            padding: 20px;
        }

        .label {
            padding-left: 0px;
            font-size: 20px;
        }

        .input {
            padding: 5px;
            width: 30%;
        }

        .center-button {
            margin-top: 20px;
            background-color: #cc9900;
            color: white;
            border: 2px solid black;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            border-radius: 0;
            cursor: pointer;
        }

        .center-button[disabled] {
            background-color: #ffd966;
            cursor: not-allowed; /* optional: change the cursor to indicate the button is disabled */
        }


        .start-button {
            margin-right: 10px;
            background-color: #339933;
            color: white;
            border: 2px solid black;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            cursor: pointer;
        }

        .start-button[disabled] {
            background-color: #8cd98c;
            cursor: not-allowed; /* optional: change the cursor to indicate the button is disabled */
        }

        .stop-button {
            margin-left: 10px;
            background-color: #cc3300;
            border: 2px solid black;
            color: white;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            cursor: pointer;
        }

        .stop-button[disabled] {
            background-color: #ff8c66;
            cursor: not-allowed; /* optional: change the cursor to indicate the button is disabled */
        }

        .update-button {
            margin-left: 10px;
            background-color: #00ccff;
            border: 2px solid black;
            color: black;
            padding: 15px 32px;
            text-align: center;
            text-decoration: none;
            display: inline-block;
            font-size: 16px;
            cursor: pointer;
        }

        .canvas-terminal-container {
            display: flex;
            margin-top: 20px;
            justify-content: space-between;
            align-items: center;
        }

        #graphCanvas {
            max-width: 90%;
            height: auto;
            margin-top: 20px;
            border: 2px solid red;
        }

        #dataTerminal {
            overflow-y: scroll;
            height: 500px;
            width: 300px;
            margin-left: 40px;
            border: 2px solid black;
            background-color: #ffd966;
            color: black;
            margin-top: 20px;
            font-family: monospace;
            font-size: 15px;
        }

    </style>
</head>

<body>
    <h1>omniLidar Web Control</h1>
    <form method="POST" action="/">
        <div class="container">
            <div>
                <label class="label" for="lidarTime">Lidar - Time of measure [ms]:</label>
                <input class="input" type="number" id="lidarTime" name="lidarTime">
            </div>

            <div>
                <label class="label" for="measTimePerRev [ms]">Meas - Time of Revolution:</label>
                <input class="input" type="number" id="measTimePerRev" name="measTimePerRev">
            </div>
        </div>
        <div class="container">
            <div>
                <label class="label" for="lidarMaxDistance">Lidar - Max meas distance [ms]:</label>
                <input class="input" type="number" id="lidarMaxDistance" name="lidarMaxDistance">
            </div>
            <div>
                <label class="label" for="measNumPerRev">Meas - Num per revolution:</label>
                <input class="input" type="number" id="measNumPerRev" name="measNumPerRev">
            </div>
        </div>
        <button type="button" class="center-button start-button" id="startBtn">Start</button>
        <button type="submit" class="center-button">Save</button>
        <button type="button" class="center-button stop-button" id="stopBtn" disabled>Stop</button>
    </form>

    <div class="canvas-terminal-container">
        <canvas id='graphCanvas' width='500' height='500'></canvas>
        <div id="dataTerminal"></div>
    </div>

    <form action = "/serverIndex" method = "GET">
        <button type="submit" class="center-button update-button">Update Firmware</button>
    </form>

    <script>

        var lidarMaxDistance = 2000;  // Initial default value

        var canvas = document.getElementById('graphCanvas');
        var ctx = canvas.getContext('2d');

        function drawTemplate(){

            ctx.fillStyle = 'lightgray';
            ctx.fillRect(0, 0, canvas.width, canvas.height);
            ctx.fillStyle = 'blue';

            // Draw coordinate axis and circles
            var centerX = canvas.width / 2;
            var centerY = canvas.height / 2;
            var radius = Math.min(centerX, centerY) - 20;

            // Draw coordinate axis
            ctx.strokeStyle = 'black';
            ctx.beginPath();
            ctx.moveTo(centerX, 0);
            ctx.lineTo(centerX, canvas.height);
            ctx.stroke();

            ctx.beginPath();
            ctx.moveTo(0, centerY);
            ctx.lineTo(canvas.width, centerY);
            ctx.stroke();

            // Draw lines at 45 degrees
            ctx.strokeStyle = 'green';
            ctx.beginPath();
            ctx.moveTo(centerX - radius, centerY + radius);
            ctx.lineTo(centerX + radius, centerY - radius);
            ctx.stroke();

            ctx.beginPath();
            ctx.moveTo(centerX - radius, centerY - radius);
            ctx.lineTo(centerX + radius, centerY + radius);
            ctx.stroke();


            // Draw circles representing radians
            ctx.strokeStyle = 'gray';
            var distanceBetweenCircles = lidarMaxDistance / 5;
            var textdistance = 0;
            for (var i = 1; i <= 5; i++) {
                ctx.beginPath();
                ctx.arc(centerX, centerY, radius * (i / 5), 0, 2 * Math.PI);
                ctx.stroke();
                ctx.fillText(Math.round(textdistance) + "m", centerX + (radius - 20) * (i / 5), centerY-5);
                textdistance += distanceBetweenCircles;
            }
        }
        

        function clearCanvas(){
            ctx.clearRect(0, 0, canvas.width, canvas.height);
        }

        function appendToTerminal(data) {
            const terminal = document.getElementById("dataTerminal");
            const entry = document.createElement("div");
            entry.textContent = data;
            terminal.appendChild(entry);
            terminal.scrollTop = terminal.scrollHeight; // Scroll to bottom
        }

        function drawPoints(points) {
            clearCanvas();
            drawTemplate();
            
            const scaleFactor = canvas.width / (2*lidarMaxDistance);

            appendToTerminal("New Data:");
            for(let i = 0; i < points.length; i++) {
                const x = (points[i][0] * Math.cos(points[i][1])) * scaleFactor + canvas.width / 2; 
                const y = (points[i][0] * Math.sin(points[i][1])) * scaleFactor + canvas.height / 2;

                ctx.beginPath();
                ctx.arc(x, y, 5, 0, 5 * Math.PI); 
                ctx.fillStyle = 'red'; 
                ctx.fill();
                appendToTerminal(`Dis: ${points[i][0].toFixed(2)}, Ang: ${points[i][1].toFixed(2)}`);
            }
        }

        var socket = new WebSocket('ws://' + location.hostname + ':81/');
        socket.onmessage = function(event) {
            const data = event.data;
            
            if (data.startsWith("lidarMaxDistanceUpdate:")) {
                // Extract and update the lidarMaxDistance
                lidarMaxDistance = parseInt(data.split(":")[1], 10);
                drawTemplate();  // Redraw the template with updated value
            } else if (data === "isStopped") { // if isStopped is true
                document.querySelector(".center-button[type='submit']").disabled = false;
                document.querySelector(".stop-button[type='button']").disabled = true;
                document.querySelector(".start-button[type='button']").disabled = false;
            } else if (data === "isStarted") {
                document.querySelector(".center-button[type='submit']").disabled = true;
                document.querySelector(".stop-button[type='button']").disabled = false;
                document.querySelector(".start-button[type='button']").disabled = true;
                
            } else if (data !== "noCommand") {
                const pointStrings = data.split("|");
                const pointArray = pointStrings.map(str => {
                  const coords = str.split(",");
                  return [parseFloat(coords[0]), parseFloat(coords[1])];
                });
                drawPoints(pointArray);
            }
        }

        document.getElementById('startBtn').addEventListener('click', function() {
            socket.send('start');
        });

        document.getElementById('stopBtn').addEventListener('click', function() {
            socket.send('stop');
        });

        // Handle form submission
        document.querySelector("form").addEventListener('submit', function(event) {
            event.preventDefault();  // prevent default form submission

            // Construct the form data object
            var formData = {
                lidarTime: document.getElementById("lidarTime").value,
                measTimePerRev: document.getElementById("measTimePerRev").value,
                lidarMaxDistance: document.getElementById("lidarMaxDistance").value,
                measNumPerRev: document.getElementById("measNumPerRev").value
            };

            // Send the form data as a JSON string through the WebSocket
            socket.send(JSON.stringify(formData));
        });
    </script>
</body>

</html>
)";

/*
 * Server Index Page
 */
const char *serverIndex = R"(
    <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>

    <form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>
        <input type='file' name='update'>
        <input type='submit' value='Update'>
    </form>

    <div id='prg'>progress: 0%</div>

    <script>
        $('form').submit(function(e){
            e.preventDefault();
            var form = $('#upload_form')[0];
            var data = new FormData(form);
            $.ajax({
              url: '/update',
              type: 'POST',
              data: data,
              contentType: false,
              processData:false,
              xhr: function() {
                  var xhr = new window.XMLHttpRequest();
                  xhr.upload.addEventListener('progress', function(evt) {
                      if (evt.lengthComputable) {
                          var per = evt.loaded / evt.total;
                          $('#prg').html('progress: ' + Math.round(per*100) + '%');
                      }
                  }, false);
                  return xhr;
              },
              success:function(d, s) {
                  console.log('success!')
              },
              error: function (a, b, c) {}
            });
        });
    </script>
)";