// UHTS Heater Controller by Marcin Morawski (09/2017)
// Contact me at s1512783@sms.ed.ac.uk if you encounter any issues

// This sketch is based on a Finite State Machine template.
// The controller is set up as a MODBUS slave device. See modbusMap.txt
// for detailed mapping of registers.

// Libraries
#include <PID_v1.h>
#include <SPI.h>
#include <Adafruit_MAX31856Kelvin.h>
#include <Adafruit_MAX31855Kelvin.h>
#include "EmonLib.h"
#include <SimpleModbusSlave.h>


//--------------------------------------------------------------------------------------//
//  Define state machine states                                                         //
//--------------------------------------------------------------------------------------//
const int S_IDLE = 1; // Idle state
const int S_INIT = 2; // Initialisation
const int S_OPERATIONAL = 3;  // Operational state
const int S_WAIT = 5; // Wait state
const int S_EXIT = 6; // Exit state
const int S_SHUTDOWN = 7; // Emergency shutdown state
const int S_WARNINGMON = 9; // State for monitoring a warning if it occurs

//--------------------------------------------------------------------------------------//
//  Define temperature limits for warning/shutdown                                      //
//--------------------------------------------------------------------------------------//
const int WARN_SURFACE = 323;
const int MAX_SURFACE = 348; // surface temperature limits
const int WARN_HEATER = 1000;
const int MAX_HEATER = 1100; // heater temperature limits (thermocouple placed near the heater itself)

//--------------------------------------------------------------------------------------//
//  Timing parameters                                                                   //
//--------------------------------------------------------------------------------------//
const int TEMP_UPDATE = 300; //time between temperature measurements and updating data in registers

//--------------------------------------------------------------------------------------//
//  Define pins                                                                         //
//--------------------------------------------------------------------------------------//

//Thermocouple SPI (bitbanged in software, may be switched to hardware SPI later on)
const int CLK = 22; // clock pin for SPI interfacing with surface thermocouple
const int DO = 23; // digital output for reading SPI interface with thermocouple
const int DI = 24; // digital in for writing to max312856 thermocouples
const int CS_SURF = 25; // chip select for surface thermocouple
const int CS_CORE = 27; // chip select for core thermocouple
const int CS_HEATER = 26; // chip select for heater thermocouple

// Relay pins
const int PIN_RELAY = 52;

// Heater control
#define PIN_HEATERCONT DAC0
#define PIN_CURRENT A0
#define PIN_VOLTAGE A1

// RS-485 writeEnable pin (active HIGH)
const int RS485WE = 2;

//--------------------------------------------------------------------------------------//
//  Construct library stuff                                                             //
//--------------------------------------------------------------------------------------//

// Thermocouples with (CLK,CS, DO) pins (MAX31855)
Adafruit_MAX31855 therSurf(CLK, CS_SURF, DO);
// Thermocouples with (CS, DI, DO, CLK) pins (MAX31856)
Adafruit_MAX31856 therCore = Adafruit_MAX31856(CS_CORE, DI, DO, CLK);
Adafruit_MAX31856 therHeater = Adafruit_MAX31856(CS_HEATER, DI, DO, CLK);

//Set up PID variables
double PIDsetpointTemp, PIDinputTemp, PIDoutputTemp;
double KpTemp = 100, KiTemp = 0.001, KdTemp = 18000; // Temperature PID tuning parameters
PID tempPID(&PIDinputTemp, &PIDoutputTemp, &PIDsetpointTemp, KpTemp, KiTemp, KdTemp, DIRECT); // construct PID object

// Create power monitor instance
EnergyMonitor powermon1;

//--------------------------------------------------------------------------------------//
//  MODBUS Configuration                                                                //
//--------------------------------------------------------------------------------------//
enum
{
  // The first register starts at address 0
  isOnReg,
  errorTherSurfReg,
  errorTherHeatReg,
  errorTherCoreReg,
  tempSetpointReg,
  tempSurfReg,
  tempHeaterReg,
  tempCoreReg,
  powerInReg,
  TOTAL_ERRORS,  // leave this one
  TOTAL_REGS_SIZE // total number of registers for function 3 and 16 share the same register array
};

unsigned int holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array
////////////////////////////////////////////////////////////

// Set up miscellaneous

void setup() {
  /* Modbus constructor
    parameters(long baudrate,
                unsigned char ID,
                unsigned char transmit enable pin,
                unsigned int holding registers size,
                unsigned char low latency)
  */

  modbus_configure(57600, 10, 2, TOTAL_REGS_SIZE, 0);
  holdingRegs[isOnReg] = 0; // Initialise register to 0

  // Pin modes
  pinMode(LED_BUILTIN, OUTPUT); //Define built-in LED for thoubleshooting
  pinMode(PIN_RELAY, OUTPUT);

  // Initialise MAX31856 thermocouples
  therCore.begin();
  therHeater.begin();
  therCore.setThermocoupleType(MAX31856_TCTYPE_K);
  therHeater.setThermocoupleType(MAX31856_TCTYPE_K);

  // Increase ADC and DAC resolution to 12-bit from default 10-bit (probably useless due to noise)
  analogReadResolution(12);
  analogWriteResolution(12);

  // wait for MAX chips to stabilise
  delay(500);

  //Initialise heater PID
  tempPID.SetOutputLimits(0, 4095); // set output limits for PID. We are useing a 12-bit DAC, so they are from 0 to 4095
  tempPID.SetSampleTime(1000); // the PID library is also a state machine. It only evaluates the inputs/outputs at intervals specified by SampleTime
  PIDinputTemp = therCore.readThermocoupleTemperatureKelvin(); // Just initial temperatures - the input temperature is measured, and room temperature is the initial setpoint
  PIDsetpointTemp = therCore.readThermocoupleTemperatureKelvin();
  tempPID.SetMode(MANUAL); // turn PID off until needed

  // Initialise power consumption monitoring
  //  powermon1.voltage(2, 234.26, 1.7);  // Voltage: input pin, calibration, phase_shift
  //  powermon1.current(1, 111.1);       // Current: input pin, calibration.
  //  analogReadResolution(ADC_BITS) // Set ADC resolution to 12 bits for better accuracy
  SerialUSB.begin(57600); // Serial for debugging
  SerialUSB.println("Starting system");
  SerialUSB.println("In idle");
}

void loop() {

  // Some more initialisation (statics)
  static int state = S_IDLE; // Initial state
  static unsigned long timeSend = 0; // will be used to time sending data to LabVIEW
  static unsigned long timeCurrent;  

  
  
  switch (state) {

    case S_IDLE:
      {
        // Wait for initialisation from labview
        holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
        byte isOn = holdingRegs[isOnReg];
        if (isOn) {
          state = S_INIT;
          SerialUSB.println("Initialising");
        }
        break;
      }

    case S_INIT:
      {
        digitalWrite(PIN_RELAY, HIGH); // turn on relay
        //do other init stuff
        tempPID.SetMode(AUTOMATIC); // Start PID controller
        state = S_OPERATIONAL;
        SerialUSB.println("Operational");
        break;
      }

    case S_OPERATIONAL:
      {
        digitalWrite(LED_BUILTIN, HIGH); // to indicate it's operational
        // Start timing for temperature operation
        timeCurrent = millis();

        holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs); // Update modbus registers

        // Read temperature data, compare with safety limits and save to register every TEMP_UPDATE ms

        if (timeCurrent >= timeSend + TEMP_UPDATE) {

          //--------------------------------------------------------------------------------------//
          //  A) Thermocouple interfacting: read temperatures and compare them to safety limits.  //
          //--------------------------------------------------------------------------------------//

          double tempSurf = therSurf.readKelvin();
          holdingRegs[tempSurfReg] = (int) tempSurf * 65535 / 1100;
          double tempCore = therCore.readThermocoupleTemperatureKelvin();
          holdingRegs[tempCoreReg] = (int) tempCore * 65535 / 1100;
          double tempHeater = therHeater.readThermocoupleTemperatureKelvin();
          holdingRegs[tempHeaterReg] = (int) tempHeater * 65535 / 1100;

          // Check surface temperature limits. If an error occurs, go into monitoring state (so that transient errors/spikes
          // do not result in an unnecessary shutdown

          if (isnan(tempSurf) || tempSurf >= WARN_SURFACE || tempSurf >= MAX_SURFACE) { // check if thermocouple is present (not NaN) and temp. warnings are not exceeded
            state = S_WARNINGMON;
            break;
          }

          byte faultHeater = therHeater.readFault();

          if (faultHeater & MAX31856_FAULT_OPEN || tempHeater > WARN_HEATER ||  tempHeater > MAX_HEATER) { // check if thermocouple is present (not FAULT_OPEN) and temp. warnings are not exceeded
            state = S_WARNINGMON;
            break;
          }

          byte faultCore = therCore.readFault();

          if (faultCore & MAX31856_FAULT_OPEN) { // check if thermocouple is present (not FAULT_OPEN and temp. warnings are not exceeded)
            state = S_WARNINGMON;
            break;
          }

          //--------------------------------------------------------------------------------------//
          //  B) PID operation                                                                    //
          //--------------------------------------------------------------------------------------//
          
          PIDsetpointTemp = holdingRegs[tempSetpointReg];
          PIDinputTemp = tempCore; // set PID input
          tempPID.Compute(); // calculate PID outputs
          analogWrite(PIN_HEATERCONT, PIDoutputTemp); // output the control value to the heater controller
          SerialUSB.print("Heater output");
          SerialUSB.println(PIDoutputTemp);
          
          //--------------------------------------------------------------------------------------//
          //  C) Energy monitor calculations                                                      //
          //--------------------------------------------------------------------------------------//

          //       float Power      = emon1.realPower;
          float power = 1500*PIDoutputTemp/4095;
          holdingRegs[powerInReg] = (int) power * 65535 / 2500;

          timeSend = timeCurrent;
        }
         //Check if should be turned off
        if (holdingRegs[isOnReg]==0) {
          state = S_EXIT;
          break;
        }
        break;
      }


    case S_WARNINGMON:
      {
        // once a warning has occured, monitor temperature for 2s to see if the error was just a one-off. If not and the error has to do with
        // a thermocouple not detecting or critical temperature being exceeded, go into shutdown
        SerialUSB.println("Warning");

        int t_startmon = millis();
        bool errorSurf = false;
        bool warnSurf = false;
        bool critSurf = false;
        bool errorHeat = false;
        bool warnHeat = false;
        bool critHeat = false;
        bool errorCore = false;

        // monitor for 2 sec

        while (millis() < t_startmon + 2000) {

          double tempSurf = therSurf.readKelvin();
          double tempHeater = therHeater.readThermocoupleTemperatureKelvin();
          byte faultHeater = therHeater.readFault();
          byte faultCore = therCore.readFault();


          if (isnan(tempSurf)) { // this means that a thermocouple has not been detected
            errorSurf = true;
          }
          if (tempSurf > WARN_SURFACE) { // temperature approaching dangerous limit
            warnSurf = true;
          }
          if (tempSurf > MAX_SURFACE) { // temperature over critical limit
            critSurf = true;
          }
          if (faultHeater & MAX31856_FAULT_OPEN) { // this means that a thermocouple has not been detected
            errorHeat = true;
          }
          if (tempHeater > WARN_HEATER) { // temperature approaching dangerous limit
            warnHeat = true;
          }
          if (tempHeater > MAX_HEATER) { // temperature over critical limit
            critHeat = true;
          }
          if (faultCore & MAX31856_FAULT_OPEN) { // this means that a thermocouple has not been detected
            errorCore = true;
          }

        }

        if (errorSurf == true) { // this means that a thermocouple has not been detected
          holdingRegs[errorTherSurfReg] = 1;
          state = S_SHUTDOWN;
        } else if (critSurf  == true) { // temperature over critical limit
          holdingRegs[errorTherSurfReg] = 1;
          state = S_SHUTDOWN;
        }

        if (errorHeat == true) { // this means that a thermocouple has not been detected
          SerialUSB.println("TerrHeat");
          holdingRegs[errorTherSurfHeat] = 1;
          state = S_SHUTDOWN;
        } else if (warnHeat == true && critHeat == false) { // temperature approaching dangerous limit
          SerialUSB.println("TwarnHeat");
        } else if (critHeat == true) { // temperature over critical limit
          holdingRegs[errorTherSurfHeat] = 1;
          SerialUSB.println("TcritHeat");
          state = S_SHUTDOWN;
        } else {
          state = S_OPERATIONAL;
        }

        if (errorCore == true) {
          SerialUSB.println("TerrCore");
          holdingRegs[errorTherSurfCore] = 1;
          state = S_SHUTDOWN;
        }

        break;
      }
      
    case S_SHUTDOWN:
      {
        digitalWrite(PIN_RELAY, LOW); // close relay
        digitalWrite(LED_BUILTIN, LOW); // to indicate turn-off
        SerialUSB.println("Emergency shutdown");
        break;
      }

    case S_EXIT:
      {
        SerialUSB.println("Exiting");
        digitalWrite(PIN_RELAY, LOW);
        // do other exit stuff
        digitalWrite(LED_BUILTIN, LOW); // to indicate turn-off
        state = S_IDLE;
        break;
      }
  }
}


