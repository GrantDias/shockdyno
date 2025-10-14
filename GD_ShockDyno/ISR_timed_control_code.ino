/*  =============================================================================
 *  Real-Time Interrupt Service for implementing a controller on the Arduino Uno
 *
 *  Board:           Developed for Arduino Uno (16MHz base clock frequency)
 *  Description of this program:
 *  This program is a working example of how to configure and run an interrupt service routine at a
 *  desired fixed sample rate, which is required for most real-time control implementations.  This
 *  program also includes examples of how to access analog and digital inputs and outputs on the
 *  Arduino Uno.  Most of the code should run on the Arduino Uno and other Arduino platforms
 *  with minor changes to pin assignments.
 *
 *  Definitions of some useful Acronyms:
 *    IDE = Integrated Development Environment, the software development interface
 *    ISR = Interrupt Service Routine, the real-time control primary function
 *    RT = real-time
 *    SPI = Serial Peripheral Interface, a 4-wire serial communication bus protocol
 *    CAN = Controller Area Network, a 2-wire serial communication bus protocol
 *    UART = Universal Asynchronous Receiver/Transmitter, chip to translate between parallel & 2-wire serial
 *    I/O = Input/Output
 *    PWM = Pulse Width Modulation, these are used to drive output channels
 *    LED = Light Emitting Diode
 *    Hz = sampling frequency units, also known as "Hertz", also known as "samples/second"
 *    DC = literally means "Direct Current", but here refers to "zero frequency", or "constant"
 *    SRAM = Static Random Access Memory
 *    EEPROM = Electrically Eraseable PRogrammable Memory
 *
 *  There are three types of memory on the Arduino.
 *    1) Flash memory: this is where your program is stored.  This memory is non-volatile,
 *        which means that your program stays in this memory even after the power is turned off.
 *        The Mega has 256K of flash memory.  Flash memory can only be written ~10000 times.
 *    2) SRAM:  this is where your program creates and manipulates variables during run time.
 *        SRAM is volatile, which means that this memory is cleared when power is turned off.
 *        The Mega has 8K of SRAM memory.
 *    3) EEPROM:  this memory is also nonvolatile and can be used to store data for long-term
 *        retrieval.  The Mega has 4K of EEPROM.
 */

//  Include the header libraries

//  These header files are required to support serial communication
#include <SPI.h>
#include <SoftwareSerial.h>
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




//  These Compiler Directives define compile-time Constants used in the real-time software

//  Pin assignment definitions
#define LED_OUT     13          // LED pin number (the on-board LED is hardwired to digital pin 13 on the Uno)
#define DIG_IN      7           // Digital input pin number (can choose from 2, 4, 5, 6, 8, 9, 10)
#define DIG_OUT     8           // Digital output pin number (can choose from 2, 4, 5, 6, 8, 9, 10)
//#define PWM_OUT     3           // 8-bit PWM digital output pin number (TIMER2 is hardwired to pins 11 and 3 on the Uno)
//   PWM outputs can only generate 0V-5V output square waves with a prescribed duty cycle
#define ANA_IN      A0          // Define the pin number to read an analog input (can choose from pins A0-A5 on the Uno)
//   Analog inputs can only be 0V-5V input signals
#define DIG_PROBE   6           // Digital output pin number used for probing processor usage with an oscilloscope


#define MOTOR               11// PWM Pin the motorcontroller is connected to
#define LIN_POT_PIN         A5 //Use all caps for defines to signify text string, best practice
#define TCOUPLE_PIN         4
#define LOADCELL_DATA_PIN   3 //
#define LOADCELL_CLOCK_PIN  2

#define CALIBRATION_FACTOR -7050.5 //Found using the sparkfun calibration code
#define ZERO_FACTOR  8421804 //Found using the sparkfun calibration code

#define PRINT_TO_SERIAL_MONITOR //Must be defined in order for data to be printed to Serial monitor, comment out to stop data printing


/* NOTE:  All processors will have an upper limit for how large the sampling frequency FS can be, and this limit will
 *  depend on the amount of computation you will do in the ISR every sample, so you cannot just sample arbitrarily fast. You
 *  will have to experiment to determine the upper limit for your application, and you most likely do not need to sample
 *  as fast as possible with your hardware.  If you try to sample too fast, i.e. if FS is too big, you will "overrun the
 *  processor", which basically means that you are asking the processor to do too many computations in a fixed amount of
 *  time, which can cause your program to run incorrectly.
 */

#define FS          100        // user specified INTEGER sample rate (Hz)

//  Declare the global variables (all functions have access to global variables)
static bool control_enable = false; //Estop variable, can connect it to a digital input pin, and then to a physical switch to stop code
float realTime=0; //Records time since program started
float dataTime=0;//records time since data collection started
static bool control_data = false; //Start point for data collection

// Time barriers for motor control in seconds.

int duration1 = 10;
int duration2 = 20;
int duration3 = 30;



/* NOTE:
 *  TIMER1 is used to set the sample rate for this program.  The TIMER1 scaling in this example code only allows
 *  accurate sample rates from 1Hz to 4000Hz.  Using this example code, FS=4000Hz requires ~80% processor usage
 */

/* https://stackoverflow.com/questions/18705363/arduino-uno-pwm-pins-conflict */

/* NOTE:  A second-order low-pass digital filter has been included in this software as an example.  You may or may
 *  not need to use it, but it can often be very useful to clean up a signal.  Please note that this digital filter
 *  can NOT be used to provide anti-aliasing for your sampled signals!  Also note that this digital filter can ONLY
 *  be used on one signal.  Modification will be required if you want to filter more than one signal.  The digital 
 *  filter has a "unity gain at DC", which means that if you apply a constant input to the filter, the output of the 
 *  filter will give you the same constant value after any transients have decayed away.  Any sinusoidal or noise 
 *  signals that are greater than the "break frequency" will be rejected.
 *
 */

/*  ----------setup----------
 *
 *    This is the setup function which is automatically run one time during startup,
 *    which happens at powerup or reset.  This function should include all 
 *    initialization tasks.
 *
 *    Do NOT change the name of this function!
 */

//==============================Thermocouple setup==========================
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(TCOUPLE_PIN);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
DeviceAddress addr;
//==============================Load cell setup==========================
HX711 scale;
  createBufferedOutput(output, 66, DROP_UNTIL_EMPTY); //Creates 66byte buffer to store and print data without delay, might need to move out of the setup() function

void setup()
{
  //  Copyright  (c)   2017  Steve Southward <scsouth@vt.edu>

  //  Declare variables local to the setup function


  //  Initialize the serial monitor (required if you are printing to the Serial Monitor)
  //  the entered number is called the "baud rate" which means "bits per second"
  Serial.begin(115200);  // can choose {4800, 9600, 19200, 38400, 57600, 115200, 230400, or 250000} this is how fast it comunicates with the computer
//check how to match baud rate with computer and arduino


  //  Configure the digital I/O pins we want to use
  pinMode(LED_OUT, OUTPUT); 
  pinMode(MOTOR,OUTPUT);
  pinMode(LIN_POT_PIN, INPUT);
  pinMode(TCOUPLE_PIN,INPUT);

  // Start up the library for the thermocouple
  sensors.begin();

  //Start up load cell
  scale.begin(LOADCELL_DATA_PIN, LOADCELL_CLOCK_PIN);
  scale.set_scale(CALIBRATION_FACTOR);
  //scale.tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0. This line breaks the ISR, look into it
   scale.set_offset(ZERO_FACTOR); //Works with the ISR


  //  Configure the TIMERS
  /*  see the following links for more details on configuring PWM Timers on the Arduino platform
   *  <http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/>
   *  <https://arduino-info.wikispaces.com/Arduino-PWM-Frequency>
   *  <https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/>
   *  <https://arduino-info.wikispaces.com/Timers-Arduino>
   *  <http://playground.arduino.cc/Main/TimerPWMCheatsheet>
   */

  noInterrupts();              // first disable all global processor interrupts

  //  Configure TIMER1 to drive the Interrupt Service Routine
  TCCR1A = 0;                  // initialize entire TCCR1A register to zero (upper half of a 2-byte register)
  TCCR1B = 0;                  // initialize entire TCCR1B register to zero (lower half of a 2-byte register)
  TCNT1  = 0;                  // reset the initial Timer1 CouNT value
  if (FS >= 10)  {  // configure for higher sample rates
    OCR1A = (int)(62500 / FS); // define the total counts in the Output Control Register to achieve FS Hz
    TCCR1B |= (1 << WGM12);    // set the WGM12 bit to enable Clear Timer on Compare (CTC) mode
    TCCR1B |= (1 << CS12);     // set the CS12 bit
    //  the CS12 bit creates a prescaled 16MHz/256=62.5kHz base clock
  } else {          // configure for lower sample rates
    OCR1A = (int)(15625 / FS); // define the total counts in the Output Control Register to achieve FS Hz
    TCCR1B |= (1 << WGM12);    // set the WGM12 bit to enable Clear Timer on Compare (CTC) mode
    TCCR1B |= (1 << CS12);     // set the CS12 bit
    TCCR1B |= (1 << CS10);     // set the CS10 bit
    //  the CS12 and CS10 bits create a prescaled 16MHz/1024=15.625kHz base clock
  }
  TIMSK1 |= (1 << OCIE1A);     // enable the timer Output Compare Interrupt bit OCIE1A


  //  Configure TIMER2 to drive the PWM's (for generating analog outputs in pins 11 and 3 on the Uno)
  TCCR2B &= 0b11111000;        // set bits 7, 6, 5, 4, and 3 to define the PWM waveform generator mode
  TCCR2B |= (1 << CS11);       // set the CS11 bit
  //  the CS11 bit defines a divisor of 8 to create a prescaled 16MHz/510/8 = 3921.57 Hz base PWM frequency

  interrupts();                // now it is ok to enable all global processor interrupts
}


/*  ----------ISR----------
 *
 *   This is the Interrupt Service Routine function.  This function will be called at
 *   the fixed uniform sample rate of FS Hz.  All of the real-time functional tasks
 *   that must run at a fixed clock frequency should be implemented inside this function.
 *
 *   Do NOT change the name of this function!
 */

ISR(TIMER1_COMPA_vect)
{
  //  Copyright  (c)   2017  Steve Southward <scsouth@vt.edu>

  //  Declare variables local to the ISR function
  static int samplenum = 0;
//Declare local variables for all the data colleciton before printing to serial monitor.
  float loadCell=0;
  float lin_pot=0;
  float T_couple=0;

  //  Toggle the LED heartbeat (this will switch the LED on and off every second), operation duration: ?
  samplenum++;  //  increment the counter
  realTime=realTime+(1/FS);
  
  if (samplenum == (int)FS)
  { 
    //  one second has just completed
    samplenum = 0;  //  reset the counter
    digitalWrite(LED_OUT, !digitalRead(LED_OUT));  //  toggle the LED 
  }
  //Read data
//Repeat for each datacollection 

loadCell=scale.get_units();
lin_pot=analogRead(LIN_POT_PIN); //Need to scale data
sensors.requestTemperatures();// Send the command to get temperatures
T_couple=sensors.getTempCByIndex(0);//replace with index of thermocouple


  if (T_couple>85 && dataTime<duration3) //Once shock temperature reaches value, data collection starts 
  {
    control_data=true;
  }

  if        ( control_data==true )//start data collecting something
  {
  
  dataTime=dataTime+(1/FS);   //add to the timecounter
  
//Define cycle counter as global variable near FS. 
    //  Note:  the value of control_out MUST be an 8-bit unsigned integer in the range of 0 to 255
    //  because the PWM output is only 8-bit, so it can only accept values between 0 and 255
    //control_out = constrain(control_out, 0, 255); //  saturate the control_out value between 0 and 255, makes sure the value out is never lower or higher than set values
    
    //Use the ifdefine to preface all of the output
    #ifdef PRINT_TO_SERIAL_MONITOR
    output.print(dataTime); //print time since data recording started
    output.print(",");
    //Load Cell data
    output.print(loadCell, 1); //scale.get_units() returns a float,prints load cell value
    output.print(",");
    //Thermocouple
    output.print(T_couple);
    output.print(",");
    //Linear Potentiometer
    output.print(lin_pot);//Prints lin potentiometer data, change to local variable, need to scale
    output.println("");//Print to new line
    #endif
      }
}


/*  ----------loop----------
 *
 *   This is the main program loop.  The only tasks that should be in this loop are background tasks
 *   that do not need to run at the main sampling frequency FS.  Your may not have any background
 *   tasks, or you may include serial communication or diagnostic monitoring of signals here.  An
 *   example State Machine is included in the loop function below.  State Machines are commonly used
 *   to manage sequencing of different processes in a control system.
 *
 *   Do NOT change the name of this function!
 */


void loop()
{

  //  Define local variables
  static int state = 0;
  static int count = 0;
  bool dig_in_value = false;


  //  Run all background tasks

  //  Read a Digital Input pin, used for enabling the control process
  dig_in_value = digitalRead(DIG_IN); 
  //  dig_in_value will either be false (logical LOW) or true (logical HIGH)


  //  Evaluate the state machine, This is a coded E-stop keep this in the code. Maybe install an LED to indicate the state
  switch (state)
  {
    case 0:   //  This is the wait-to-start state
      //  Disable control
      control_enable = false;
    
      //  Write a digital output, not required, only an example...
      digitalWrite(DIG_OUT, control_enable); //  operation duration: ~7.8us

      //  Wait for user to enable control
      if (dig_in_value == true)
      {
        //  reset the count and move to the next state
        count = 0;
        state = 1;
      }
      break;
      
    case 1:    //  This is the counting state
      //  Debounce the digital input signal
      if (dig_in_value == true)
      {
        //  increment the count
        count++;

        //  wait for 50 consecutive enables
        if (count == 50) state = 2;  // move to the next state
      }
      else
      {
        //  return to the wait-to-start state
        state = 0;
      }
      break;

    case 2:   //  This is the initialize/enable state
    
    
      //  enable control
      control_enable = true;
    
      //  Write a digital output, not required, only an example...
      digitalWrite(DIG_OUT, control_enable); //  operation duration: ~7.8us

      //  move to the next state
      state = 3;
      break;
      
    case 3:   //  This is the wait-to-stop state
      //  Keep checking for the dig-in_value to be false
      if (dig_in_value == false)  state = 0;  // move to the wait-to-start state
      break;

    default:    //  Should never be here, but just in case...
        //  move to the wait-to-start state
        state = 0;
        break;
  }
  
//===============Motor controls====================== 
  if (realTime < 3 )//for the first 3 seconds of the program the motor will run slowly
  {
    analogWrite(MOTOR, 100);
  }

 while (control_data==false)// while the shock is less than set temeprature, motor will run at ~half throttle
{
      analogWrite(MOTOR, 120);
}

if (control_data==true && dataTime < duration1)
{
        analogWrite(MOTOR, 170);

}

if (dataTime > duration1 && dataTime < duration2)
{
        analogWrite(MOTOR, 200);

}

if (dataTime > duration2 && dataTime< duration3)
{
        analogWrite(MOTOR, 230);

}

  if (dataTime > duration3) //after time has passed planned runtime, motor stops and data collection stops
{
        analogWrite(MOTOR, 0);
      control_data=false;
}
}
