// -----------------------------------------------------------------------------
// JeeNode for Use with BMP085 and LuxPlug
// reads out a BMP085 sensor connected via I2C
// see http://news.jeelabs.org/2010/06/20/battery-savings-for-the-pressure-plug/
// see http://news.jeelabs.org/2010/06/30/going-for-gold-with-the-bmp085/
//
// Baesd on RoomNode form JeeLabs roomNode.pde
//
// 2010-10-19 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: FHEM_JSN_LUX.pde,v 1.1 2011-07-19 09:31:20 rudolfkoenig Exp $
//
// see http://jeelabs.org/2010/10/20/new-roomnode-code/
// and http://jeelabs.org/2010/10/21/reporting-motion/
// -----------------------------------------------------------------------------
// Includes
#include <JeeLib.h>
#include <PortsSHT11.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h> //BH1750 IIC Mode 
#include <math.h> 
//-------------------------------------------------------------------------------
int BH1750address = 0x23; //setting i2c address LUX
byte buff[2]; //buffer Lux
static long payload2;
 uint16_t val=0; 
//One-Wire config
// Data wire is plugged into pin 4 (P1DIO) on the Arduino
#define ONE_WIRE_BUS 9

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// Assign the addresses of your 1-Wire temp sensors.
// See the tutorial on how to obtain these addresses:
// http://www.hacktronics.com/Tutorials/arduino-1-wire-address-finder.html

//DeviceAddress insideThermometer = { 0x28, 0x99, 0xD2, 0x83, 0x03, 0x00, 0x00, 0x51 };

// -----------------------------------------------------------------------------
// JeeNode RF12-Config
byte myNodeID = 5;   // node ID used for this unit
static byte myNetGroup = 212; // netGroup used for this unit
//unsigned long lux; +++
// Port Lux-Plug +++
//#define LUX_PORT 4 +++
int reedp = 8;
// Payload aka Data to Send
struct  {
    // RF12LowBat
    byte rf12lowbat_type;
    byte rf12lowbat_data;
    // Lux +++
    //byte lux_type; +++
    //unsigned long lux_data; +++
    //One Wire Temp
    byte temp_type;
    int temp_data;
    
} payload;
// -----------------------------------------------------------------------------
// Lux Plug
//PortI2C two (LUX_PORT); ++
//LuxPlug lsensor (two, 0x39); ++
//byte highGain; ++
//MilliTimer timer; ++
// -----------------------------------------------------------------------------
// Config & Vars
#define SERIAL  1   // set to 1 to also report readings on the serial port
#define DEBUG   0   // set to 1 to display each loop()
#define MEASURE_PERIOD  70 // how often to measure, in tenths of seconds
#define RETRY_PERIOD    10  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     5   // maximum number of times to retry
#define ACK_TIME        10  // number of milliseconds to wait for an ack
#define REPORT_EVERY    1   // report every N measurement cycles
#define SMOOTH          3   // smoothing factor used for running averages

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2
// -----------------------------------------------------------------------------
// The scheduler makes it easy to perform various tasks at various times:
enum { MEASURE, REPORT, TASK_END };

static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

static byte reportCount;    // count up until next report, i.e. packet send

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (rf12_recvDone() && rf12_crc == 0 &&
                rf12_hdr == (RF12_HDR_DST | RF12_HDR_ACK | myNodeID))
            return 1;
        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }
    return 0;
}

// readout all the sensors and other values
static void doMeasure() //DeviceAddress deviceAddressA
    {
    // RF12lowBat
    payload.rf12lowbat_type = 253;
    payload.rf12lowbat_data = rf12_lowbat();
    
   
    // lux_demo.pde
    // need to wait after changing the gain
    //  see http://talk.jeelabs.net/topic/608
    // highGain = ! highGain;
    // lsensor.setGain(highGain);
    // Sleepy::loseSomeTime(1000);
    // Lux Plug
    //const word* p = lsensor.getData(); +++
    //lux = lsensor.calcLux(); +++
    //payload.lux_type = 12; +++
    //payload.lux_data = lux; +++
    // payload.lux_data = lsensor.calcLux();
    payload.temp_type = 11;
    sensors.requestTemperatures();
    float tempA = sensors.getTempCByIndex(0);
    //float tempA = sensors.getTempC(deviceAddressA);
   
    int itempa = 10 * tempA;
    payload.temp_data = itempa;
    //payload.temp_data = //sensors.getTempCByIndex(0);
    digitalWrite(5, HIGH);  
    digitalWrite(12, HIGH);
    int i;
    
    delay(120);
    BH1750_Init(BH1750address);
    delay(120);
    if(2==BH1750_Read(BH1750address))
   { 
    val=((buff[0]<<8)|buff[1])/1.2;
    //uint16_t lux = 0;
    //payload.lux_type = 12; 
    //payload.lux_data = lux; +++
    //payload.temp_data = val;
    Serial.print(val,DEC);     
    Serial.println("[lx]");
   } 
    
   digitalWrite(12, LOW);
   digitalWrite(5, LOW);
  
  }
// periodic report, i.e. send out a packet and optionally report on serial port
static void doReport() {
    //myNodeID = 5;   
    rf12_sleep(-1);
    while (!rf12_canSend())
        rf12_recvDone();
    rf12_sendStart(0, &payload, sizeof payload, RADIO_SYNC_MODE);
    Sleepy::loseSomeTime(300);
    //myNodeID = 6; 
    payload.rf12lowbat_type = 253;
    payload.rf12lowbat_data = rf12_lowbat();
    payload.temp_type = 12;
    //int itempa = 10 * tempA;
    payload.temp_data = val;
    payload.temp_data = digitalRead(8);
    rf12_sendStart(0, &payload, sizeof payload, RADIO_SYNC_MODE);
    rf12_sleep(0);

    #if SERIAL
        Serial.print("ROOM PAYLOAD: ");
        Serial.print("RF12LowBat: ");
        Serial.print((int) payload.rf12lowbat_data);
        //Serial.print(" L: "); +++
        //Serial.print(payload.lux_data); +++
        Serial.print(" T:");
        Serial.print(payload.temp_data);
        Serial.print(" = ");
        Serial.print((int) payload.temp_data);
        Serial.println();
        delay(2); // make sure tx buf is empty before going back to sleep
    #endif
}
// -----------------------------------------------------------------------------
void setup () {
    pinMode(reedp, INPUT);
    Wire.begin();
    rf12_initialize(myNodeID, RF12_868MHZ, myNetGroup);
    #if SERIAL
        Serial.begin(57600);
        Serial.print("\n[FHEM-Onewiretemp]");
       // myNodeID = rf12_config();
    #endif

    rf12_sleep(0); // power down
    // Start Lux-Plug
    sensors.begin();
    //lsensor.begin();+++
    Sleepy::loseSomeTime(1000);
   // sensors.setResolution(insideThermometer, 10);
    //highGain = 1;++
    // highGain = ! highGain;
    //lsensor.setGain(highGain);++
    Sleepy::loseSomeTime(1000);
    reportCount = REPORT_EVERY;     // report right away for easy debugging
    scheduler.timer(MEASURE, 0);    // start the measurement loop going
}
// -----------------------------------------------------------------------------
void loop () {
    #if DEBUG
        Serial.print('.');
        delay(2);
    #endif
    
    switch (scheduler.pollWaiting()) {

        case MEASURE:
            // reschedule these measurements periodically
            scheduler.timer(MEASURE, MEASURE_PERIOD);
              
            doMeasure();//insideThermometer

            // every so often, a report needs to be sent out
            if (++reportCount >= REPORT_EVERY) {
                reportCount = 0;
                scheduler.timer(REPORT, 0);
            }
            break;

        case REPORT:
            doReport();
            break;
    }
}
// -----------------------------------------------------------------------------
int BH1750_Read(int address) //
{
  int i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) //
  {
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
  Wire.endTransmission();  
  return i;
}
void BH1750_Init(int address) 
{
  Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();
}
