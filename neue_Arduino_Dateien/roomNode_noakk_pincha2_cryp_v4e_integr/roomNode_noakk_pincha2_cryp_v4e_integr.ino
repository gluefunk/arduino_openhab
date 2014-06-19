/// @dir roomNode
/// New version of the Room Node (derived from rooms.pde).
// 2010-10-19 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// see http://jeelabs.org/2010/10/20/new-roomnode-code/
// and http://jeelabs.org/2010/10/21/reporting-motion/
// The complexity in the code below comes from the fact that newly detected PIR
// motion needs to be reported as soon as possible, but only once, while all the
// other sensor values are being collected and averaged in a more regular cycle.
#include <JeeLib.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include "DHT.h"
#include <Wire.h> //BH1750 IIC Mode = Lux
#include <math.h> 
#include <OneWire.h>
#include <DallasTemperature.h>

#define SERIAL  1   // set to 1 to also report readings on the serial port
#define DEBUG   1   // set to 1 to display each loop() run and PIR trigger
#define SERIAL_BAUD  9600
#define NODEID        5 //network ID used for this unit
#define NETWORKID    212  //the network ID we are on
#define GATEWAYID     1  //the node ID we're sending to

#define MEASURE_PERIOD  40 // how often to measure, in tenths of seconds
#define RETRY_PERIOD    5  // how soon to retry if ACK didn't come in
#define RETRY_LIMIT     5   // maximum number of times to retry
#define ACK_TIME        50  // number of milliseconds to wait for an ack
#define REPORT_EVERY    1   // report every N measurement cycles
#define SMOOTH          2   // smoothing factor used for running averages
// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

#define AKKU_MODE      0 // 0/1 = no / AKKU_MODE
#define AKKU+ACK_DAT_MODE 0 // 0/1 = no / AKKU_MODE with Databack when Ack
#define LED_PIN        3 // 3  P3 the pin that the LED is attached to
#define LOWBAT         1 // 0 =kein // 1 Battery in mV
#define LOWBATDOWN     0//3630  //mV
#define LOWBATALARM    0 //3670  //mV
#define INPPORT        0 // 9  pin 9 = reedkontakt/switch to Ground as Input
#define SPEAKER_PIN    0 // ex Piezospeaker (no Res.)
#define SPEAKER_COUNT  4
#define SERVO_PIN      0 // 3/5/6 PWM run only with 5V !!
#define DS18B20_PORT1  8 // 8 defined if DS18B20 is connected to a port
#define DS18B20_PORT2  0 // 8 if DS18B20 is connected to the same port
#define DS18B20_PORT3  0 // 8
#define DS18B20_PORT4  0 // 8
#define LUX_PORT       0 // 1 if vcc connected to VCC / 5 if Luxboards VCC is connected through the port pin5
#define LDR_PORT       2 // 2 ok !! if LDR is connected to the port A2 pin and Ground
#define PIR_PORT       1 // 1+3= D4  if PIR / Reedkontakt to D4 Interrupt and Ground 
#define INTERRUPTX2    0 // 1 interrupted if PIR_PORT change to - and to + takes >0.15mA 
#define ONE_WIRE_BUS  DS18B20_PORT1// any OneWire devices  port 
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire
DallasTemperature sensors(&oneWire);// Pass oneWire reference to Dallas Temp
#define DHTPIN         0 //7
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
#include <Servo.h>
Servo servo;
String c ;
char input = 0;
unsigned long time;
byte speak=0 ;
Port ldr (LDR_PORT);
int BH1750address = 0x23; // i2c address
byte buff[2];
String dd; String aa;
// The scheduler makes it easy to perform various tasks at various times:
enum { MEASURE, REPORT, TASK_END };
static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);
// Other variables used in various places in the code:
byte measurecount;
static byte reportCount;    // count up until next report, i.e. packet send
static byte myNodeID = NODEID;       // node ID used for this unit
int ak;
char payload3[63];
byte n;
// This defines the structure of the packets which get sent out by wireless:
struct {
    int lobat ;//:1;  // supply voltage dropped under 3.1V: 0..1
    byte light;     // light sensor: 0..255
    byte moved ;//:1;  // motion detector: 0..1
    int humi ;// :7;  // humidity: 0..100
    int temp  ;// :10; // temperature: -500..+500 (tenths)
    int temp0;int temp1;
    int temp2;int temp3;
    int val  ;
} payload;
// Conditional code, depending on which sensors are connected and how:
#if PIR_PORT
    #define PIR_HOLD_TIME   1  // hold PIR value this many seconds after change
    #define PIR_PULLUP      1   // set to one to pull-up the PIR input pin
    #define PIR_INVERTED    0   // 0 or 1, to match PIR reporting high or low    
    /// Interface to a Passive Infrared motion sensor.
    class PIR : public Port {
        volatile byte value, changed;
        volatile uint32_t lastOn;
    public:
        PIR (byte portnum)
            : Port (portnum), value (0), changed (0), lastOn (0) {}
        // this code is called from the pin-change interrupt handler
        void poll() {
            // see http://talk.jeelabs.net/topic/811#post-4734 for PIR_INVERTED
            byte pin = digiRead() ^ PIR_INVERTED;
            // if the pin just went on, then set the changed flag to report it
            if (pin) {
                if (!state())
                    changed = 1;
                lastOn = millis(); }
           // }
            value = pin;
        }
        // state is true if curr value is still on or if it was on recently
        byte state() const {
            byte f = value;
            if (lastOn > 0)
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    if (millis() - lastOn < 1000 * PIR_HOLD_TIME)
                        f = 1; }                
            return f;
        }
        // return true if there is new motion to report
        byte triggered() {
            byte f = changed;
            changed = 0;
            return f;  }        
    };
    PIR pir (PIR_PORT);
    // the PIR signal comes in via a pin-change interrupt (to -)
    ISR(PCINT2_vect) { pir.poll(); }
#endif

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }
//_________________________________________________________________________

void setup () {
    #if LED_PIN
    pinMode (LED_PIN, OUTPUT);
    #endif
    servo.attach(SERVO_PIN);
    pinMode(INPPORT, INPUT);
    sensors.begin(); Wire.begin();
    rf12_encrypt(RF12_EEPROM_EKEY);
    #if DHTPIN
      dht.begin();
    #endif
    #if SERIAL || DEBUG    
        Serial.begin(SERIAL_BAUD);
        Serial.print("\n[roomNode.3]");
        rf12_initialize(NODEID, RF12_868MHZ, NETWORKID);        
        serialFlush();           
    #endif    
    rf12_sleep(RF12_SLEEP); // power down    
    #if PIR_PORT
        pir.digiWrite(PIR_PULLUP);
      #ifdef PCMSK2
        bitSet(PCMSK2, PIR_PORT + 3);
        bitSet(PCICR, PCIE2);    
      #endif
    #endif
    #if  !AKKU_MODE
      time = millis()+(MEASURE_PERIOD*100);
      measurecount = 0;
    #endif
    
    reportCount = REPORT_EVERY;     // report right away for easy debugging
    scheduler.timer(MEASURE, 0);    // start the measurement loop going    
}
//____________________________________________________________________________

void loop () {
    #if SPEAKER_ALARM 
     if speak  tone(SPEAKER_PIN, 2800, 500);      
    #endif
    #if DEBUG
       //Serial.print('.');
       Serial.flush();
    #endif
    #if PIR_PORT
        if (pir.triggered()) {
            payload.moved = pir.state();Serial.println();
            Serial.print(pir.state());Serial.print(" !");
            doTrigger();  }        
    #endif 
    #if  !AKKU_MODE
       if (rf12_recvDone() && rf12_crc == 0) {     
           byte n = rf12_len;
           #if DEBUG
            Serial.println();
            Serial.println(rf12_data[n-6]);Serial.print("-");
            Serial.print(rf12_data[n-7]); Serial.print("-");
            Serial.print(rf12_data[n-2]);//Serial.print("-");
            Serial.println();
            //Serial.print(rf12_data[6]);Serial.print("-"); Serial.println();
           #endif 
           byte led = rf12_data[n-6]-48 ;          
           if (led >=3 && led <=9) {
             if (rf12_data[n-7]== 49 ) {led =led+10 ;}
             #if DEBUG
             Serial.println(led);
             #endif
             pinMode (led, OUTPUT); 
             byte val = rf12_data[n-2]-48;
             if (val == 1) digitalWrite(led, HIGH);
             if (val == 0) digitalWrite(led, LOW); 
             if (val >= 2) analogWrite(led, (val-1)*val*3);} 
           if( rf12_data[n-6]==48) { 
             doMeasure(); 
             doReport();  }       
           #if DEBUG  
             Serial.flush();
             Serial.println(rf12_data[n-2]);Serial.print("+");Serial.print(rf12_hdr);Serial.print("-");
             Serial.print(RF12_HDR_ACK);Serial.print("-");Serial.print(RF12_HDR_DST);Serial.print("-");
             Serial.print(RF12_WANTS_ACK);Serial.print("-");Serial.print(RF12_ACK_REPLY);Serial.print("+");
             Serial.print(rf12_len);Serial.print("+");Serial.print( RF12_HDR_CTL);
             for (byte i = 0; i < n; ++i) {
              Serial.print((char)rf12_data[i]);} 
             Serial.flush();     
            #endif     
      } 
    #endif 
    #if  !AKKU_MODE
     if (time <= millis()) {
       //Serial.print(timem); Serial.print(millis());
       doMeasure();
       measurecount ++;             
       time = millis()+(MEASURE_PERIOD*100);
       if (measurecount >= REPORT_EVERY) {doReport(); measurecount=0;}        
      }      
    #endif      
    #if  AKKU_MODE
    switch (scheduler.pollWaiting()) {        
        case MEASURE: // reschedule these measurements periodically           
            scheduler.timer(MEASURE, MEASURE_PERIOD);    
            doMeasure();
            // every so often, a report needs to be sent out
            if (++reportCount >= REPORT_EVERY) {
                reportCount = 0;
                scheduler.timer(REPORT, 0);  }            
            break;            
        case REPORT:
            doReport();
            break;     }
    #endif     
} //End loop
//_____________________________________________________________________________

// Utility code to perform simple smoothing as a running average

static int smoothedAverage(float prev, float next, byte firstTime =0) {
    if (firstTime)
        return next;
    return ((SMOOTH - 1) * prev + next + SMOOTH / 2) / SMOOTH;
} 
// readout all the sensors and other values
static void doMeasure() {
    byte firstTime = payload.humi = payload.temp = payload.temp0 == 0;   
    // special case to init running avg            
    #if LOWBATALARM
     uint16_t bat =  readVcc();     
     if(bat <= LOWBATALARM)  
     { speak++;
     if (speak >SPEAKER_COUNT) speak=0 ;}      
    #endif
    #if LOWBATDOWN
      uint16_t bat2 =  readVcc();
      // RADIO_SYNC_MODE =3; ..powerdown
      if(bat2 <= LOWBATDOWN)  Sleepy::loseSomeTime(60000);//MEASURE_PERIOD= 600
     #endif
    #if DS18B20_PORT1 //////////
      sensors.requestTemperatures();
      int temp0 =  sensors.getTempCByIndex(0)*10;
      payload.temp0 = smoothedAverage(payload.temp0, temp0, firstTime);        
    #endif 
    #if DS18B20_PORT2 //////////
      sensors.requestTemperatures();
      int temp1 =  sensors.getTempCByIndex(1)*10;
      payload.temp1 = smoothedAverage(payload.temp1, temp1, firstTime);     
    #endif
    #if DS18B20_PORT3 //////////
      sensors.requestTemperatures();
      temp2 =  sensors.getTempCByIndex(2)*10;
      payload.temp2 = smoothedAverage(payload.temp2, temp2, firstTime);     
    #endif 
    #if DS18B20_PORT4 //////////
      sensors.requestTemperatures();
      temp3 =  sensors.getTempCByIndex(3)*10;
      payload.temp3 = smoothedAverage(payload.temp3, temp3, firstTime);     
    #endif
    #if DHTPIN
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      int humi = h*10 ;
      int temp = t*10 ;     
      payload.humi = smoothedAverage(payload.humi, humi, firstTime);
      payload.temp = smoothedAverage(payload.temp, temp, firstTime);      
    #endif   
    #if LDR_PORT
        ldr.digiWrite2(1);  // enable AIO pull-up  = A4
        byte light = ldr.anaRead();//>> 2;
        Serial.print(light);
        ldr.digiWrite2(0);  // disable pull-up to reduce current draw
        payload.light = smoothedAverage(payload.light, light, firstTime);        
    #endif
    #if LUX_PORT  //////////////
    digitalWrite(LUX_PORT, HIGH);   //  digitalWrite(5, HIGH);  
    delay(120);
    int i; uint16_t val=0;     
    BH1750_Init(BH1750address); delay(120);  
    if(2==BH1750_Read(BH1750address))
    { val=((buff[0]<<8)|buff[1])/1.2;  } 
    payload.val = smoothedAverage(payload.val, val, firstTime);   
    #if DEBUG //////////////
     Serial.print(val,DEC);     
     Serial.print("[lx]"); //&payload
    #endif    //////////////
    digitalWrite(LUX_PORT, LOW); //digitalWrite5, LOW);    

   #endif    //////////////    
    #if PIR_PORT
        payload.moved = pir.state();
    #endif    
    //Serial.print(payload.light);    
}    
// periodic report, i.e. send out a packet and optionally report on serial port
static void doReport() {
    rf12_sleep(RF12_WAKEUP);
    aa = "i=";aa += NODEID;
    #if LOWBAT
     uint16_t bat =  readVcc();     
     aa += ";b="; aa += bat;
     //payload.lobat = bat;  //rf12_lowbat(); 
    #endif    
    #if DS18B20_PORT1
     float cc = payload.temp0/10.0; char dd[4];
     dtostrf(cc,4,1,(dd));    
     aa += ";t="; aa += dd;      
    #endif
    #if DS18B20_PORT2
     cc = payload.temp1/10.0;  dd[4];
     dtostrf(cc,4,1,(dd));    
     aa += ";u="; aa += dd;
    #endif
    #if DS18B20_PORT3
     cc = payload.temp2/10.0;  dd[4];
     dtostrf(cc,4,1,(dd));    
     aa += ";v="; aa += dd;       
    #endif
    #if DS18B20_PORT4
     cc = payload.temp3/10.0; dd[4];
     dtostrf(cc,4,1,(dd));    
     aa += ";w="; aa += dd;
    #endif
    #if DHTPIN     //serialFlush();Serial.println();
     float cd = payload.humi/10.0; char de[4];
     dtostrf(cd,4,1,(de));    
     aa += ";H="; aa += de;
     cd = payload.temp/10.0; de[4];
     dtostrf(cd,4,1,(de));
     aa += ";T="; aa += de;
    #endif
    #if INPPORT 
     if (digitalRead(INPPORT)) {aa += ";s=1";}
     else aa += ";s=0"; 
    #endif 
    #if PIR_PORT 
     aa += ";P=";aa += payload.moved ; 
    #endif
    #if LDR_PORT
      aa += ";L="; aa += payload.light;
    #endif
    #if LUX_PORT
      aa += ";l="; aa += payload.val;
    #endif
    if (c)  {
      aa += c;
      c = "";}
    Serial.println( aa); 
    ak = aa.length()+1; payload3[ak]; 
    aa.toCharArray(payload3,ak );     
    #if DEBUG //////////////    
     serialFlush();//Serial.println();
     //Serial.print(aa);//Serial.print(ak);       
    #endif     //////////////

    rf12_sendNow(RF12_HDR_ACK | RF12_HDR_DST |1, &payload3, ak );
        rf12_sendWait(RADIO_SYNC_MODE);
        byte acked = waitForAck();
        //rf12_sleep(RF12_SLEEP);static long payload;
        if (acked) {
            #if SERIAL
                Serial.print(" ack");
                //Serial.println((int) i);
                serialFlush();
            #endif
            }
    //rf12_sendWait(RADIO_SYNC_MODE);
    #if  AKKU_MODE
     rf12_sleep(RF12_SLEEP);
    #endif
    #if SERIAL
       /* Serial.print("  ROOM ");
        Serial.print(myNodeID);
        Serial.print(' ');              
        serialFlush();  */
    #endif
}
// send packet and wait for ack when there is a motion trigger
static void doTrigger() {
    #if DEBUG
        Serial.print("PIR ");
        Serial.print((int) payload.moved);
        serialFlush();
    #endif
     //for (byte i = 0; i < RETRY_LIMIT; ++i) {
       // rf12_sleep(RF12_WAKEUP);
        //doMeasure();
        //serialFlush();Serial.println("- ");
        //Serial.print((int) payload.moved);Serial.print(" ");
        c += ";c=1";
        doReport();
        //Serial.print((int) payload.moved);Serial.print(" *");
        //serialFlush();
        //rf12_sendNow(RF12_HDR_ACK, &payload3, ak);
        /*rf12_sendWait(RADIO_SYNC_MODE);
        byte acked = waitForAck();
        rf12_sleep(RF12_SLEEP);
        if (acked) {
            #if DEBUG
                Serial.print(" ack ");
                Serial.println((int) i);
                serialFlush();
            #endif
            // reset scheduling to start a fresh measurement cycle  
            scheduler.timer(MEASURE, MEASURE_PERIOD);
            return;  }                
        delay(RETRY_PERIOD * 100); 
    }*/
    scheduler.timer(MEASURE, MEASURE_PERIOD);
    #if DEBUG
       // Serial.println(" no ack!");
        //serialFlush();
    #endif
}
// spend a little time in power down mode while the SHT11 does a measurement
//static void shtDelay () {
//    Sleepy::loseSomeTime(32); }// must wait at least 20 ms

// wait a few milliseconds for proper ACK to me, return true if indeed received
static byte waitForAck() {
    MilliTimer ackTimer;
    while (!ackTimer.poll(ACK_TIME)) {
        if (AKKU_MODE && rf12_recvDone() && rf12_crc == 0   ) {  //&& 
          /// bei akku-version hier Zustandsänderungen z.B.Measureperiod beep etc.
          if (rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))                
            Serial.println("**");Serial.println(RF12_HDR_DST | RF12_HDR_CTL | myNodeID);
            n = rf12_len;Serial.print(n);
            byte led = rf12_data[n-6]-48 ;          
            if (led >=3 && led <=9) {
             if (rf12_data[n-7]== 49 ) {led =led+10 ;}
              //Serial.println(led);
              pinMode (led, OUTPUT); 
              byte val = rf12_data[n-2]-48;
              if (val == 1) digitalWrite(led, HIGH);
              if (val == 0) digitalWrite(led, LOW); 
              if (val >= 2) analogWrite(led, (val-1)*val*3);} 
             if( rf12_data[n-6]==48) { 
               doMeasure(); 
               doReport();  }       
             #if DEBUG  
               Serial.flush();
               Serial.println(rf12_data[n-2]);Serial.print("+");Serial.print(rf12_hdr);Serial.print("-");
               Serial.print(RF12_HDR_ACK);Serial.print("-");Serial.print(RF12_HDR_DST);Serial.print("-");
               Serial.print(RF12_WANTS_ACK);Serial.print("-");Serial.print(RF12_ACK_REPLY);Serial.print("+");
               Serial.print(rf12_len);Serial.print("+");Serial.print( RF12_HDR_CTL);
               for (byte i = 0; i < n; ++i) {
                Serial.print((char)rf12_data[i]);} 
               Serial.flush();     
             #endif 
             /*
                 for (byte i = 0; i < n; ++i) {
                 Serial.print(' ');
                 Serial.print(rf12_data[i]);  }
                 //// bei akku-version hier Zustandsänderungen z.B.Measureperiod beep etc.
                 //&& rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
                 Serial.print("*");Serial.println("-");
                 Serial.print(RF12_HDR_ACK);Serial.print("-");Serial.print(RF12_HDR_DST);Serial.print("-");
                 Serial.print(RF12_HDR_CTL);Serial.print("-");Serial.print(rf12_hdr);
                 Serial.println("*");
                 */
            return 1; }
          }
        //set_sleep_mode(SLEEP_MODE_IDLE);
        //sleep_mode();    
    return 0;  }

static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif  
    delay(2);  }// make sure tx buf is empty before going back to sleep
   
void blink (byte pin) {
    for (byte i = 0; i < 6; ++i) {
        delay(100);
        digitalWrite(pin, !digitalRead(pin)); }  
}        
// Luxmeter
int BH1750_Read(int address) { 
  int i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) //
  {buff[i] = Wire.read();  // receive one byte    
    i++;  } 
  Wire.endTransmission();  
  return i;                  }
//----
void BH1750_Init(int address)     {
Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();         }
//--- 
long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


