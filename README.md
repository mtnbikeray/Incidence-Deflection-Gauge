# Incidence-Deflection-Gauge
RC incidence &amp; deflection gauge.  Similar to Gliderthrow a commercial product.
I started making an incidence gauge but found I could add a second sensor and be able to measure deflection of ailerons, flaps, rudder.
It seems to be accurate to within 0.1 degree. The sensors attach to the control surface with a magnet.
Simply attaching one of the sensors to the incidence gauge allows you to see wing or horiz stab incidence.
The incidence gauge was a remix of https://www.thingiverse.com/thing:1730475 to make it fit an old heli boom I had laying around.
The sensor brackets are a remix of https://www.thingiverse.com/thing:2911596 with a small rare earth magnet epoxied to the top.  another magnet under the control surface will hold things in place
There's lots of Wemos D1 mini cases on thingiverse

Its based on a Wemos D1 mini (esp8266) and 2 MPU6050 sensors. Electronics cost less than $15.
The only others parts needed are a couple of 20mm bolts, the 3d printed parts, some wire and plugs and a boom.
Much of the code is based on https://github.com/adesandr/MAD_Single
It connects to your Wifi using the Autoconnect library by Hieromon Ikasamo https://github.com/Hieromon/AutoConnect , allowing you to enter you wifi credentials then displays the sensor info as a web page on your smart phone.
Each sensor/magnet weighs about 6 grams and my version of the incidence gauge with the sensor attached is about 50 grams.
Both sensors connect as follows
vcc to 5v (wemos)
gnd to gnd (wemos)
scl to d1 (wemos)
sda to d2 (wemos)
sensor 2 needs a jumper from vcc to AD0

The first time you connect the gauge it’ll create an access point.
It needs to be powered through a micro usb plug. 
After about a minute you should see “esp8266ap” as an available network on your smart phone.
Connect to it and it should present you with a login screen
If it asks for a password use 12345678
You should see an autoconnect login screen,  Select the three horizntal bars (hamburger menu?)
and select Configure New AP.  Select your network and enter your credentials and you should be presented with a information screen showing the current IP address
of the Wemos Mini.  Make note of this address to be used to connect to the gauge.  It should be something like 192.168.1.xx
This only needs to be done once.
Restartng the device will connect to the same network.
If you have trouble finding the device, look at all devices connected to your router,

This current version needs a parameter to set the chord value.
in your browser. when you connect to the sensor use the following format

192.168.1.xx/?c=nn       where nn is the chord size of the control surface in millimeters and 192.168.1.xx is the address given when you entered your network credentials
IOS users can now use the following in place of the ip address
throws.local/?c=nn

When the device initializes it will calibrate itself and this can take about 2 minutes...during this time the BLUE led will be solid.  Leave the sensors on a flat surface during this.
Once the BLUE led starts flashing it is ready for a connection.
