/// @dir radioBlip
/// Send out a radio packet every minute, consuming as little power as possible.
// 2010-08-29 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
//#include <RFM12B.h>
#include <JeeLib.h>
#include <PortsSHT11.h> //dht11 dht22 etc = Temp/Humidity
#include <avr/sleep.h>
#include <util/atomic.h>
#include <Wire.h> //BH1750 IIC Mode = Lux
#include <math.h> 
#include <OneWire.h> // DS18B20 = Temp
#include <DallasTemperature.h>  // DS18B20

#define SERIAL     1
#define NODEID     7
#define INPPORT       8   // 0 =kein //8 =DIO 8
#define DS18B20_PORT1  9 // defined if DS18B20 is connected to a port
#define DS18B20_PORT2  9 // defined if DS18B20 is connected to a port
#define SHT11_PORT    0   // 5 defined if SHT11 is connected to a port
#define LUX_PORT      6   // defined if LDR is connected to a port's AIO pin
#define PIR_PORT      0   // defined if PIR is connected to a port's DIO pin
#define ONE_WIRE_BUS 9 // any OneWire devices  port 4 
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire
DallasTemperature sensors(&oneWire);// Pass oneWire reference to Dallas Temp
#if SHT11_PORT
    SHT11 sht11 (SHT11_PORT);
    static void shtDelay () {
    Sleepy::loseSomeTime(32);   }// must wait at least 20 ms                            
#endif
bool requestACK=false;
float temp;
String dd; String aa;
int BH1750address = 0x23; // i2c address
byte buff[2];
static long payload;
ISR(WDT_vect) { Sleepy::watchdogEvent(); } //watchdog for low-power
//--------------------------------------------------
void setup() {  //rf12_encrypt(RF12_EEPROM_EKEY);   
    pinMode(INPPORT, INPUT);
    sensors.begin(); Wire.begin();    
    #if SERIAL
    Serial.begin(9600);
    #endif
    cli(); //CLKPR = bit(CLKPCE);       
    //CLKPR = 1; // div 2, i.e. slow down to 8 MHz
    sei();
    rf12_initialize(NODEID, RF12_868MHZ, 212);
    rf12_control(0xC040); // set low-battery level to 2.2V i.s.o. 3.1V
}
//--------------------------------------------------
void loop() {
   byte lowbat= rf12_lowbat();
   aa = ";b="; aa += lowbat;
   #if DS18B20_PORT1 //////////
   sensors.requestTemperatures();
   temp =  sensors.getTempCByIndex(0);
   aa += ";t="; aa += temp; 
   #endif          ///////////  
   #if DS18B20_PORT2 //////////
   temp =  sensors.getTempCByIndex(1);
   aa += ";u="; aa += temp; 
   #endif          /////////// 
   #if SHT11_PORT
        sht11.measure(SHT11::HUMI, shtDelay);        
        sht11.measure(SHT11::TEMP, shtDelay);
        float h, t;
        sht11.calculate(h, t);
        int humi = h + 0.5, temp = 10 * t + 0.5;
   aa += ";h="; aa += humi;
   aa += ";T="; aa += humi;
   #endif   
   #if INPPORT 
     if (digitalRead(INPPORT)) aa+=";s=1";
     else aa += ";s=0"; 
   #endif      
   #if LUX_PORT  //////////////
    digitalWrite(12, HIGH);   digitalWrite(5, HIGH);   delay(120);   
    int i; uint16_t val=0;     
    BH1750_Init(BH1750address); delay(120);  
    if(2==BH1750_Read(BH1750address))
    { val=((buff[0]<<8)|buff[1])/1.2;  }    
    #if SERIAL //////////////
     Serial.print(val,DEC);     
     Serial.println("[lx]"); //&payload
    #endif    //////////////
    digitalWrite(12, LOW); digitalWrite(5, LOW);    
    aa+=";l="; aa+=val;
   #endif    //////////////    
    int al = aa.length(); char payload[al+1]; 
    aa.toCharArray(payload,al+1 );     
    #if SERIAL //////////////
      Serial.println("D " + aa);        
    #endif     //////////////
    rf12_sendStart(1, &payload , sizeof payload); // RF12_HDR_ACK  //dtostrf(temp, 6,2, payload2);
    //  sync mode to 2 if default /mode 3 (full powerdown)only with 258 CK startup fuses       
    rf12_sendWait(2);  rf12_sleep(RF12_SLEEP);  
    Sleepy::loseSomeTime(6000);
    rf12_sleep(RF12_WAKEUP);
 } 
//--------------------------------------------------
int BH1750_Read(int address) { 
  int i=0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while(Wire.available()) //
  {buff[i] = Wire.read();  // receive one byte    
    i++;  } 
  Wire.endTransmission();  
  return i;                  }
//--------------------------------------------------
void BH1750_Init(int address)     {
Wire.beginTransmission(address);
  Wire.write(0x10);//1lx reolution 120ms
  Wire.endTransmission();         }
 
