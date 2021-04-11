# Incidence-Deflection-Gauge
RC incidence &amp; deflection gauge
I started making an incidence gauge but found I could add a second sensor and be able to measure deflection of ailerons, flaps, rudder.
It seems to be accurate to within 0.1 degree. The sensors attach to the control surface with a magnet.
Simply attaching one of the sensors to the incidence gauge allows you to see wing or horiz stab incidence.
The incidence gauge was a remix of https://www.thingiverse.com/thing:1730475 to make it fit an old heli boom I had laying around.

Its based on a Wemos D1 mini (esp8266) and 2 MPU6050 sensors. Electronics cost less than $15.
The only others parts needed are a couple of 20mm bolts, the 3d printed parts, some wire and plugs and a boom.
It connects to your Wifi using the Autoconnect library by Hieromon Ikasamo https://github.com/Hieromon/AutoConnect , allowing you to enter you wifi credentials then displays the sensor info as a web page on your smart phone.
Each sensor/magnet weighs about 6 grams and my version of the incidence gauge with the sensor attached is about 50 grams.
