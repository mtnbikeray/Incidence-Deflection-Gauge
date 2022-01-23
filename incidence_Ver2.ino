/*
  
  HandlePortal.ino, Example for the AutoConnect library.
  Copyright (c) 2018, Hieromon Ikasamo
  https://github.com/Hieromon/AutoConnect
  This software is released under the MIT License.
  https://opensource.org/licenses/MIT
*/
/*
  This is a way of not explicitly declaring ESP8266WebServer. It uses
  the ESP8266WebServer function without its declaration.
  I recommend that you consider this example compared to HandlePortalEX.ino.
  https://github.com/Hieromon/AutoConnect/blob/master/examples/HandlePortalEX/HandlePortalEX.ino
  It will help you understand AutoConnect usage.
*/

//  Version 2 01/11/22 save calibration data in nvm  rjc
// Modified by Bearded Flyer 18 Oct 2021

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WebServer.h>
#endif
// prototype hack  nvm may call serial port
class SerialBegin { public: SerialBegin() { Serial.begin(115200); } }; SerialBegin serialbegin;


#include <Nvm.h>
 // The layout and constructor for saving accelerometer & gyro offsets in NVM
static NvmField fields[] = {
  {"x1"    , "0"     , 5, 0},
  {"y1"    , "0"     , 5, 0},
  {"z1"    , "0"     , 5, 0},
  {"gx1"   , "0"     , 5, 0},
  {"gy1"   , "0"     , 5, 0},
  {"gz1"   , "0"     , 5, 0},
  {"x2"    , "0"     , 5, 0},
  {"y2"    , "0"     , 5, 0},
  {"z2"    , "0"     , 5, 0},
  {"gx2"   , "0"     , 5, 0},
  {"gy2"   , "0"     , 5, 0},
  {"gz2"   , "0"     , 5, 0},


  {0         , 0                             ,  0, 0}, // Mandatory sentinel
};

Nvm nvm(fields);


#include<Wire.h>
#include <SPI.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#define LED 2
#define DELAY_MEASURE 10                  // Measure period
#define DELAY_DISPLAY 250                 // Display period


MPU6050 accelgyro;
MPU6050 accelgyro2(0x69);

const int MPU_addr=0x68;
//set ADO to high (3.3v) for 0x69
const int MPU_addr2=0x69;


int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
//int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int gyro_x2, gyro_y2, gyro_z2;
long gyro_x_cal2, gyro_y_cal2, gyro_z_cal2;
int startup = 0;
int16_t ax, ay, az;                       // raw measure
int16_t gx, gy, gz;
uint8_t Accel_range;
uint8_t Gyro_range;
float travel = 0.0;
float angle=0.0, angleY=0.0;
float travel2 = 0.0;
float angle2=0.0, angle2Y=0.0;

float a1f,a2f,t1f,t2f,a1fz,a2fz,t1fz,t2fz;
float Y1f,Y2f,y1fz,y2fz;
float angle1_zero=0.0,angle2_zero=0.0,travel1_zero=0.0,travel2_zero=0.0,y1_zero=0.0,y2_zero=0.0;
float Da1f,Da2f,Dt1f,Dt2f;
float elapsed_time,newtime,timeprev;          // Timers for angle measurment
long t1 = 0;                              // Timer for Display management
long t2 = 0;                              // Timer for measure
int calib_in_progress = 0;
int chordControlSurface = 50;             // Chord in mm. 50 mm by default.
String T_chord;
String ip_addr;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int sw=0;
boolean set_gyro_angles;
boolean set_gyro_angles2;
char auth[] = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";

//  prototype reset_sensor  avoid compile error
void reset_sensor();

#include <AutoConnect.h>

#ifndef BUILTIN_LED
#define BUILTIN_LED  2  // backward compatibility
#endif

AutoConnect portal;
AutoConnectConfig config;

void handleRoot() {
WebServerClass& server = portal.host();
T_chord = server.arg("c");  
if (T_chord != "") {
  chordControlSurface = T_chord.toInt();
  }
  
  String page = PSTR(
"<html>"
"<head>"
  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">"
  "<meta http-equiv=\"refresh\" content=\"1\">"
  "<style type=\"text/css\">"
    "body {"
    "-webkit-appearance:none;"
    "-moz-appearance:none;"
    "font-family:'Arial',sans-serif;"
    "text-align:center;"
    "}"
    ".menu > a:link {"
    "position: absolute;"
    "display: inline-block;"
    "right: 12px;"
    "padding: 0 6px;"
    "text-decoration: none;"
    "}"
    ".button {"
    "display:inline-block;"
    "border-radius:7px;"
    "background:#73ad21;"
    "margin:0 10px 0 10px;"
    "padding:10px 20px 10px 20px;"
    "text-decoration:none;"
    "color:#000000;"
    "}"
  "</style>"
    "<style>"
"table, th, td {"
"font-size: 20px;"
"padding: 6px;"
"  border: 1px solid black;"
"  border-collapse: collapse;"
"}"
"table.center {"
"  margin-left:auto; "
"  margin-right:auto;"
"}"
"</style>"
"</head>"
"<body>"

"<div class=\"menu\">" AUTOCONNECT_LINK(BAR_32) "</div>"
"<script>"
" function getData() {"
"  var xhttp = new XMLHttpRequest();"
"  xhttp.onreadystatechange = function() {"
"    if (this.readyState == 4 && this.status == 200) {"
"      document.getElementById(\"RedXValue\").innerHTML ="
"      this.responseText;"
"   }"
" };"
"  xhttp.open(\"GET\", \"readXValue\", true);"
"  xhttp.send();"
"}"
"</script>" 

  "DEFLECTION<br>"
  "GAUGE");
page += String(F("<p><a class=\"button\" href=\"/zerosensor\">Zero Sensors</a></p>"));
page += String(F("<font size=\"+3\">"));
page += String(F("<br><br> Chord &nbsp;&nbsp;" ));page += String(chordControlSurface);page += String(F("<br><br>"));
page += String(F("<table class=\"center\"><thead><tr><th> </th><th>Sensor 1</th><th>Sensor 2</th></tr></thead>"));
page += String(F("<tbody><tr><td>"));page += String(F("X Ang (deg)</td><td> <span style=\"font-weight:bold;color:Tomato\">"));page += String(a1f, 1);page += String(F("</span></td><td>")); 
page += String(a2f, 1);page += String(F("</td>"));page += String(F("</tr>"));
page += String(F("<tr><td>"));page += String(F("Trav (mm) </td><td>  <span style=\"font-weight:bold;color:Tomato\">"));page += String(t1fz, 1);page += String(F("</span></td><td>")); 
page += String(t2fz, 1);page += String(F("</td>"));page += String(F("</tr>"));
page += String(F("<tr><td>"));page += String(F("Y Ang (deg)</td><td>  <span style=\"font-weight:bold;color:Tomato\">"));page += String(Y1f, 1);page += String(F("</span></td><td>")); 
page += String(Y2f, 1);page += String(F("</td>"));page += String(F("</tr>"));
page += String(F("</tbody></table>"));
page += String(F("<br><br><br>"));
page += String(F("</font>"));
page += ip_addr;
page += String(F("<p><a class=\"button\" href=\"/calb\">Calibrate Sensors</a></p>"));

page += String(F("</body></html>"));
   
portal.host().send(200, "text/html", page);
}

void ZeroSensor() {
WebServerClass& server = portal.host();

angle1_zero = angle;
angle2_zero = angle2;
y1_zero = angleY;
y2_zero = angle2Y;

/*
y1_zero = Y1f;
y2_zero = Y2f;
*/
travel1_zero = t1f;
travel2_zero = t2f;

//xxZ = xx;
//yyZ = yy;
  
  sendRedirect("/");
}

void readChord() {

WebServerClass& server = portal.host(); 
String message = "";
String chordstr = "";
if (server.arg("c")== ""){     //Parameter not found
message = "Chord Argument not found";
Serial.print("++++++++not found ");

}else{     //Parameter found
message = "Chord Argument = ";
message += server.arg("c");     //Gets the value of the query parameter
chordstr = server.arg("c");
chordControlSurface = chordstr.toInt();
Serial.print("++++++++new chord ");
 Serial.println(chordControlSurface);
}
 sendRedirect("/");        //Returns the HTTP response

}

void handleXValue() {
 WebServerClass& server = portal.host(); 
// int a = analogRead(A0);
String RedXValue;
 Serial.println(RedXValue);
 server.send(200, "text/plane", RedXValue); //Send ADC value only to client ajax request
}

void sendRedirect(String uri) {
  WebServerClass& server = portal.host();
  server.sendHeader("Location", uri, true);
  server.send(302, "text/plain", "");
  server.client().stop();
}

bool atDetect(IPAddress softapIP) {
  Serial.println("Captive portal started, SoftAP IP:" + softapIP.toString());
  return true;
}

void setup() {
  char val[10],val2[10];
  delay(1000);
//  Serial.begin(115200);
  Serial.println();
  pinMode(BUILTIN_LED, OUTPUT);

  
  // Put the home location of the web site.
  // But in usually, setting the home uri is not needed cause default location is "/".
  //portal.home("/");   

  // Starts user web site included the AutoConnect portal.
  // Declares to reserve the EEPROM_CONFIG_t area for a Sketch using.
config.boundaryOffset = sizeof(fields);
Serial.println(sizeof(fields));
portal.config(config);
  portal.onDetect(atDetect);
  if (portal.begin()) {
    WebServerClass& server = portal.host();
    server.on("/gval", readChord);
    server.on("/", handleRoot);
    server.on("/zerosensor", ZeroSensor);
    server.on("/readXValue", handleXValue);
    server.on("/specificArgs", readChord);
    server.on("/calb", reset_sensor);

    if (MDNS.begin("throws")) {
    Serial.println("MDNS started!");

     MDNS.addService("http", "tcp", 80);
    }
    else {
    Serial.println("MDNS failed.");
}
    ip_addr = WiFi.localIP().toString();
    Serial.println("Started, IP:" + WiFi.localIP().toString());
  }
  else {
    Serial.println("Connection failed.");
    while (true) { yield(); }
  }

 //accelgyro.initialize();


  Wire.begin();

Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
/*
 * the MPU-6050 needs to be calibrated before it is used for the first time. 
 * What we want to do is remove the zero-error; this refers to when the sensor records a small angle even though it is totally level. 
 * This error can be removed by applying an offset to the raw accelerometer and gyroscope sensor readings. 
 * The offset needs to be adjusted until the gyroscope readings are zero (no rotation) and the accelerometer records the acceleration 
 * due to gravity pointing directly downwards. 
 */
// check for zero in x2 & y2 accel offsets  (theres a chance one is zero, two prob not
  nvm.get("x2",val);
  nvm.get("y2",val2);
  int nvmck = atoi(val);
  int nvmck2 = atoi(val2);
  if ( (nvmck == 0) && (nvmck2 == 0) ){
    Serial.println("found zero");
  startup = 0;
//calibrate sensors and save in nvm
  reset_sensor();     
  startup = 1;
  } else {
//  load offsets from nvm & print to serial    
      startup = 1;
      nvm.get("x1",val);
      ax_offset = atoi(val);
      nvm.get("y1",val);
      ay_offset = atoi(val);
      nvm.get("z1",val);
      az_offset = atoi(val);
      nvm.get("gx1",val);
      gx_offset = atoi(val);
      nvm.get("gy1",val);
      gy_offset = atoi(val);
      nvm.get("gz1",val);
      gz_offset = atoi(val);
      
 
  
  accelgyro.setXAccelOffset(ax_offset);

  accelgyro.setYAccelOffset(ay_offset);
  
  accelgyro.setZAccelOffset(az_offset);
  
  accelgyro.setXGyroOffset(gx_offset);
  
  accelgyro.setYGyroOffset(gy_offset);
  
  accelgyro.setZGyroOffset(gz_offset);
    Serial.print("x: ");
  Serial.println(ax_offset);
    Serial.print("y: ");
  Serial.println(ay_offset);
    Serial.print("z: ");
  Serial.println(az_offset);
    Serial.print("gx: ");
  Serial.println(gx_offset);
    Serial.print("gy: ");
  Serial.println(gy_offset);
    Serial.print("gz: ");
  Serial.println(gz_offset);

Wire.beginTransmission(MPU_addr2);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);

  
  // if nvm (non volatile memory) was uninitialized update it w current values

      nvm.get("x2",val);
      ax_offset = atoi(val);
      nvm.get("y2",val);
      ay_offset = atoi(val);
      nvm.get("z2",val);
      az_offset = atoi(val);
      nvm.get("gx2",val);
      gx_offset = atoi(val);
      nvm.get("gy2",val);
      gy_offset = atoi(val);
      nvm.get("gz2",val);
      gz_offset = atoi(val);
    
  
  accelgyro2.setXAccelOffset(ax_offset);
  accelgyro2.setYAccelOffset(ay_offset);
  accelgyro2.setZAccelOffset(az_offset);
  accelgyro2.setXGyroOffset(gx_offset);
  accelgyro2.setYGyroOffset(gy_offset);
  accelgyro2.setZGyroOffset(gz_offset);
  
  Serial.print("x: ");
  Serial.println(ax_offset);
    Serial.print("y: ");
  Serial.println(ay_offset);
    Serial.print("z: ");
  Serial.println(az_offset);
    Serial.print("gx: ");
  Serial.println(gx_offset);
    Serial.print("gy: ");
  Serial.println(gy_offset);
    Serial.print("gz: ");
  Serial.println(gz_offset);
  }

  }
// ---------------------------------

/*******************************************************
 * void reset_sensor(void)
 * 
 * average sensors reading & save in nvm
 *******************************************************/
void reset_sensor(){
    char val[10];
    Serial.print("startup ");
  Serial.println(startup);
   
    accelgyro.setXAccelOffset(0);
    accelgyro.setYAccelOffset(0);
    accelgyro.setZAccelOffset(0);

    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);
    digitalWrite(LED, LOW);// turn the LED on.(Note that LOW is the voltage level but actually 

  meansensors(MPU_addr);
  delay(1000);
  calibration(MPU_addr);
  delay(1000);

  accelgyro.setXAccelOffset(ax_offset);
  itoa(ax_offset,val,10); 
  nvm.put("x1",val);
  
  accelgyro.setYAccelOffset(ay_offset);
  itoa(ay_offset,val,10); 
  nvm.put("y1",val);
  
  accelgyro.setZAccelOffset(az_offset);
  itoa(az_offset,val,10); 
  nvm.put("z1",val);
  
  accelgyro.setXGyroOffset(gx_offset);
  itoa(gx_offset,val,10); 
  nvm.put("gx1",val);
  
  accelgyro.setYGyroOffset(gy_offset);
  itoa(gy_offset,val,10); 
  nvm.put("gy1",val);
  
  accelgyro.setZGyroOffset(gz_offset);
  itoa(gz_offset,val,10); 
  nvm.put("gz1",val);
  Serial.print("x: ");
  Serial.println(ax_offset);
    Serial.print("y: ");
  Serial.println(ay_offset);
    Serial.print("z: ");
  Serial.println(az_offset);
    Serial.print("gx: ");
  Serial.println(gx_offset);
    Serial.print("gy: ");
  Serial.println(gy_offset);
    Serial.print("gz: ");
  Serial.println(gz_offset);

 
Wire.beginTransmission(MPU_addr2);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);  
    accelgyro2.setXAccelOffset(0);
    accelgyro2.setYAccelOffset(0);
    accelgyro2.setZAccelOffset(0);

    accelgyro2.setXGyroOffset(0);
    accelgyro2.setYGyroOffset(0);
    accelgyro2.setZGyroOffset(0);
  meansensors(MPU_addr2);
  delay(1000);
  calibration(MPU_addr2);
  delay(1000);
  
  accelgyro2.setXAccelOffset(ax_offset);
  itoa(ax_offset,val,10); 
  nvm.put("x2",val);
  
  accelgyro2.setYAccelOffset(ay_offset);
  itoa(ay_offset,val,10); 
  nvm.put("y2",val);
  
  accelgyro2.setZAccelOffset(az_offset);
  itoa(az_offset,val,10); 
  nvm.put("z2",val);
  
  accelgyro2.setXGyroOffset(gx_offset);
  itoa(gx_offset,val,10); 
  nvm.put("gx2",val);
  
  accelgyro2.setYGyroOffset(gy_offset);
  itoa(gy_offset,val,10); 
  nvm.put("gy2",val);

  accelgyro2.setZGyroOffset(gz_offset);
  itoa(gz_offset,val,10); 
  nvm.put("gz2",val);
   Serial.print("x: ");
  Serial.println(ax_offset);
    Serial.print("y: ");
  Serial.println(ay_offset);
    Serial.print("z: ");
  Serial.println(az_offset);
    Serial.print("gx: ");
  Serial.println(gx_offset);
    Serial.print("gy: ");
  Serial.println(gy_offset);
    Serial.print("gz: ");
  Serial.println(gz_offset);
// this code is used during init & also on button press 
// only redirect if we got here on button press
     if (startup == 1){
      sendRedirect("/");
     }
}
/*******************************************************
 * void meansensors(void)
 * 
 * average sensors reading
 *******************************************************/
void meansensors(int mpu){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;
  char val[10];

  while (i<(buffersize+101)){
 
    // read raw accel/gyro measurements from device
 //   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Wire.beginTransmission(mpu);
Wire.write(0x3B);
Wire.endTransmission(true);

Wire.requestFrom(mpu,14,true);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
ax=Wire.read()<<8|Wire.read();
ay=Wire.read()<<8|Wire.read();
az=Wire.read()<<8|Wire.read();
Tmp=Wire.read()<<8|Wire.read();
gx=Wire.read()<<8|Wire.read();
gy=Wire.read()<<8|Wire.read();
gz=Wire.read()<<8|Wire.read();

    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
 
    delay(2); //Needed so we don't get repeated measures
  }
} /* End meansensors() */

/*******************************************************
 * void calibration(void)
 * 
 * MPU6050 calibration
 *******************************************************/
void calibration(int mpu){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  
  while (1){
    int ready=0;
    if (mpu == MPU_addr)
    {
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
    }
    else {
    accelgyro2.setXAccelOffset(ax_offset);
    accelgyro2.setYAccelOffset(ay_offset);
    accelgyro2.setZAccelOffset(az_offset);

    accelgyro2.setXGyroOffset(gx_offset);
    accelgyro2.setYGyroOffset(gy_offset);
    accelgyro2.setZGyroOffset(gz_offset);
      
    }

    meansensors(mpu);
    Serial.println(".");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
} /* End calibration */
/*******************************************************
 * void manageMeasure()
 * 
 * MPU6050 measurement
 *******************************************************/
void manageMeasure(int mpu){
  
    /*--- Compute Y angle in degre. Complementary filter is used to combine accelero and gyro datas       ---*/
    /*--- see  http://www.pieter-jan.com/node/11 for more information regarding the complementary filter  ---*/
    /*--- or https://delta-iot.com/la-theorie-du-filtre-complementaire/ (in french)                       ---*/
    /*--- Basically complementary filter avoid used of kallman filter, quiet difficult to implement in    ---*/
    /*--- arduino platform. Gyro are used for fast motion as accelero are used for slow motion.           ---*/                                 
 //   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 timeprev = newtime;  // the previous time is stored as the time read
 newtime = millis(); // actual time read
 elapsed_time = (newtime - timeprev) / 1000; // elapsed time in seconds
 
// if we are re calibrating sensors do not read sensors
if (calib_in_progress == 0) {
  

 
 Wire.beginTransmission(mpu);
Wire.write(0x3B);
Wire.endTransmission(true);

Wire.requestFrom(mpu,14,true);
  
  //Following statements left shift 8 bits, then bitwise OR.  
  //Turns two 8-bit values into one 16-bit value                                       
ax=Wire.read()<<8|Wire.read();
ay=Wire.read()<<8|Wire.read();
az=Wire.read()<<8|Wire.read();
Tmp=Wire.read()<<8|Wire.read();
gx=Wire.read()<<8|Wire.read();
gy=Wire.read()<<8|Wire.read();
gz=Wire.read()<<8|Wire.read();
if (mpu == MPU_addr) {
    angle=0.98*(angle+float(gy)*elapsed_time/131) + 0.02*atan2((double)ax,(double)az)*180/PI;
    angleY=0.98*(angleY+float(gx)*elapsed_time/131) + 0.02*atan2((double)ay,(double)az)*180/PI;

    /*--- Compute Control surface travel using : 2* sin(angle/2)* chord. Angle for sinus function needs  ---*/
    /*--- to be converted in radian (angleDegre = angleRadian *(2*PI)/360)                             ---*/ 
 /*   travel = chordControlSurface * sin((angle*(2.0*PI)/360.0)/2.0) * 2.0;       */
    }
else {
    angle2=0.98*(angle2+float(gy)*elapsed_time/131) + 0.02*atan2((double)ax,(double)az)*180/PI;
    angle2Y=0.98*(angle2Y+float(gx)*elapsed_time/131) + 0.02*atan2((double)ay,(double)az)*180/PI;

    /*--- Compute Control surface travel using : 2* sin(angle/2)* chord. Angle for sinus function needs  ---*/
    /*--- to be converted in radian (angleDegre = angleRadian *(2*PI)/360)                             ---*/ 
 /*   travel2 = chordControlSurface * sin((angle2*(2.0*PI)/360.0)/2.0) * 2.0;*/
    }

}
  
} /* End manageMeasure */

/*******************************************************
 * void manageDisplay()
 * 
 * OLED Management
 *******************************************************/
void manageDisplay(){

//  int angle_aff = abs((angle-angle1_zero) * 10);
//  int travel_aff = abs(travel * 10);
//  int angle_aff2 = abs((angle2-angle2_zero) * 10);
//  int travel_aff2 = abs(travel2 * 10);
//  int angleY_aff = abs((angleY-y1_zero) * 10);
//  int angle2Y_aff = abs((angle2Y-y2_zero) * 10);

  int angle_aff = ((angle-angle1_zero) * 10);
  int travel_aff = (travel * 10);
  int angle_aff2 = ((angle2-angle2_zero) * 10);
  int travel_aff2 = (travel2 * 10);
  int angleY_aff = ((angleY-y1_zero) * 10);
  int angle2Y_aff = ((angle2Y-y2_zero) * 10);

  
  if((millis() - t1) > DELAY_DISPLAY)
  {
  if (sw == 0 ) {
       digitalWrite(LED, LOW);// turn the LED on.(Note that LOW is the voltage level but actually 
       sw = 1;
  } else {
       digitalWrite(LED, HIGH);// turn the LED off(Note that LOW is the voltage level but actually 
       sw = 0;
  }
  
  t1 = millis();

 
//  int i = 1234567890;
//float f = (i % 1000) / 1000.0f;
//f += i / 1000;

 a1f = (angle_aff % 10) / 10.0f;
 a1f += angle_aff / 10;
 a2f = (angle_aff2 % 10) / 10.0f;
 a2f += angle_aff2 / 10;
 t1f = (travel_aff % 10) / 10.0f;
 t1f += travel_aff / 10;
 t2f = (travel_aff2 % 10) / 10.0f;
 t2f += travel_aff2 / 10;

 Y1f = (angleY_aff % 10) / 10.0f;
 Y1f += angleY_aff / 10;
 Y2f = (angle2Y_aff % 10) / 10.0f;
 Y2f += angle2Y_aff / 10;


 y1fz = fabs(fabs(Y1f) - fabs(y1_zero));
 y2fz = fabs(fabs(Y2f) - fabs(y2_zero));
 a1fz = fabs(fabs(a1f) - fabs(angle1_zero));
 a2fz = fabs(fabs(a2f) - fabs(angle2_zero));

 t1fz = fabs(fabs(t1f) - fabs(travel1_zero));
 t2fz = fabs(fabs(t2f) - fabs(travel2_zero));

    /*--- Compute Control surface travel using : 2* sin(angle/2)* chord. Angle for sin function needs  ---*/
    /*--- to be converted in radian (angleDegre = angleRadian *(2*PI)/360)                             ---*/ 
    t1fz = chordControlSurface * sin((a1f*(2.0*PI)/360.0)/2.0) * 2.0;
    t2fz = chordControlSurface * sin((a2f*(2.0*PI)/360.0)/2.0) * 2.0;
    //  the following takes horizontal deflection out of consideration (special request)
//  t1fz = chordControlSurface * (sin(a1f*(2.0*PI)/360.0));
//  t2fz = chordControlSurface * (sin(a2f*(2.0*PI)/360.0));
  
  Serial.print(angle);
  Serial.print(" ");
  Serial.print(angle2);
  Serial.print(" ");
  Serial.print(travel);
  Serial.print(" ");
  Serial.println(travel2);
  Serial.print(angle1_zero);
  Serial.print(" ");
  Serial.print(angle2_zero);
  Serial.print(" ");
  Serial.print(travel1_zero);
  Serial.print(" ");
  Serial.println(travel2_zero);
  Serial.print(a1fz);
  Serial.print(" ");
  Serial.print(a2fz);
  Serial.print(" ");
  Serial.print(t1fz);
  Serial.print(" ");
  Serial.println(t2fz);

 }
 //  digitalWrite(LED, LOW);// turn the LED on.(Note that LOW is the voltage level but actually 

} /* End manageDisplay */

void loop() {
WebServerClass& server = portal.host();
server.handleClient();
MDNS.update();  
   if(millis() - t2 > DELAY_MEASURE)
  {
 
    t2 = millis();
    newtime = millis();
    manageMeasure(MPU_addr2);
    newtime = millis();
    manageMeasure(MPU_addr);
  }
  manageDisplay();
   
 
  portal.handleClient();

}
