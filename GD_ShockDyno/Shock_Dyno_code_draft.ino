#include <DallasTemperature.h>//Thermocouple library
#include <OneWire.h>//Thermocouple library

#include <HX711.h>//Load cell arduino library

#include <BufferedInput.h>
#include <BufferedOutput.h>
#include <loopTimer.h>
#include <millisDelay.h>
#include <PinFlasher.h>
#include <SafeString.h>
#include <SafeStringNameSpace.h>
#include <SafeStringNameSpaceEnd.h>
#include <SafeStringNameSpaceStart.h>
#include <SafeStringReader.h>
#include <SafeStringStream.h>
#include <SerialComs.h>

int motor=10;//Pin the motorcontroller is connected to
int lin_pot_pin=A5;
int Tcouple_pin=4;

int calibration_factor=-7050.5; //Found from the calibration code 
int loadcell_data_pin=3;
int loadcell_clock_pin=2;

unsigned long currentMillis = 0;    // stores the value of millis() in each iteration of loop()

//==============================Thermocouple setup==========================
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(Tcouple_pin);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress addr;
//==============================Load cell setup==========================
HX711 scale;

createBufferedOutput(output, 66, DROP_UNTIL_EMPTY); //Creates 66byte buffer to store and print data without delay

void setup(){
  Serial.begin(9600);
  output.connect(Serial);  // <<<<< connect the buffered output to Serial 
  //pinMode(motor,OUTPUT);
//pinMode(lin_pot_pin, INPUT);
//pinMode(Tcouple_pin,INPUT);

// Start up the library for the thermocouple
//sensors.begin();

//Start up load cell
scale.begin(loadcell_data_pin, loadcell_clock_pin);
scale.set_scale(calibration_factor);
scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
}

void loop() {
  //loopTimer.check(Serial);

currentMillis=millis();
collect_data();
run_motor();
output.nextByteOut(); // <<<<<<<<<< need to call this each loop to release next byte from buffer

//output.print
}


void collect_data(){
output.print(currentMillis);
output.print(",");
//Load Cell data
output.print(scale.get_units(), 1); //scale.get_units() returns a float,prints load cell value
output.print(",");
 //Serial.print(scale.get_units(), 1); //scale.get_units() returns a float
 //output.println(",");
//Thermocouple
/*
sensors.getTempCByIndex(0);//replace with index of thermocouple
  sensors.requestTemperatures();// Send the command to get temperatures
  output.print(sensors.getTempCByIndex(0));
  output.print(",");
//Linear Potentiometer
output.print(analogRead(lin_pot_pin));//Prints lin potentiometer data
output.println("");//Print to new line
*/
}

void run_motor(){
analogWrite(motor,255);
delay(300); //delays are in milliseconds
analogWrite(motor,0);
  
}
