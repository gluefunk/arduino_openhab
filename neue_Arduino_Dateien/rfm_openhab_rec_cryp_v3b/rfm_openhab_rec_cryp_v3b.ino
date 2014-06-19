// Configure some values in EEPROM for easy config of the RF12 later on.
// 2009-05-06 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php
// this version adds flash memory support, 2009-11-19
#include <JeeLib.h>
#include <util/crc16.h>
//#include <util/parity.h>
//#include <avr/eeprom.h>
//#include <avr/pgmspace.h>
int led = 4; // the pin that the LED is attached to
static byte myNodeID = 1; // node ID used for this unit
static byte myNetGroup = 212; // netGroup used for this unit
static byte quiet;
#define SERIAL_BAUD 9600
//#define DATAFLASH 1 // check for presence of DataFlash memory on JeeLink
//#define FLASH_MBIT 16 // support for various dataflash sizes: 4/8/16 Mbit
#define LED_PIN 4 // activity LED, comment out to disable
#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks
#define DEBUG  0
#define SERIAL 0
#include <OneWire.h>
#include <RCSwitch.h> //Lib zur Steuerung von Elro und Homeeasysteckdosen
RCSwitch mySwitch = RCSwitch();
char c0[3], c1[6], c2[3], c3[6];
String str2; String str[11];
byte ak,ii,ii2;
static unsigned long now () {
    // FIXME 49-day overflow
    return millis() / 1000; }
static void activityLed (byte on) {
  #ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, !on);
  #endif
}
void setup() {
    Serial.begin(SERIAL_BAUD);
    Serial.print("\n[RF12_Receiver.8]");
    rf12_initialize(myNodeID, RF12_868MHZ, myNetGroup);
    mySwitch.enableReceive(1); // interrupt 1 == arduino pin 3
    mySwitch.enableTransmit(9);
    mySwitch.setPulseLength(315);
    rf12_encrypt(RF12_EEPROM_EKEY);   
    pinMode(led, OUTPUT);
    digitalWrite(led, LOW);    
}
void loop() {
    //if (Serial.available())
    // handleInput(Serial.read());
    if (mySwitch.available()) {  
      Serial.println( mySwitch.getReceivedValue() );  
      long value = mySwitch.getReceivedValue();      
      //Serial.print( mySwitch.getReceivedValue() );         
      if (value == 327957) {
        Serial.print("Received ");
        Serial.println( mySwitch.getReceivedValue() );
        //Serial.print("Unknown encoding");
        mySwitch.switchOn("11110", "10000");
        mySwitch.switchOn("11110", "10000"); }     
      if (value == 327956) {
        mySwitch.switchOff("11110", "10000");
        mySwitch.switchOff("11110", "10000");
        Serial.print("Received ");
        Serial.println( mySwitch.getReceivedValue() );  }   
     mySwitch.resetAvailable(); 
    } 
    if (Serial.available() > 0) {
     Serial.readBytes(c0, 1); Serial.readBytes(c1, 5);Serial.readBytes(c2, 1); Serial.readBytes(c3, 5); 
     // einfache Lösung um seriellen Datenstrom in Variablen zu speichern
     if (c0[0] == 65 ) {mySwitch.switchOn(c1, c3); }  // c0[0] = "A"         
     if (c0[0] == 66 ) {mySwitch.switchOff(c1, c3); } // c0[0] = "B" 
     if (c0[0] == 68 )  {  
       // c0 = "D"; c1 ="i5;p"; c2="="; c3 ="3;d=1";
       str2="P=";  str2 += c2 ; str2 += c3; // str2 += c1;
       Serial.println (str2);
       ii2= (byte)c1[1] -48;
       Serial.print(ii2);
       //requestACK = false;
       ak = str2.length()+1; char payload2[ak]; 
       str2.toCharArray(payload2,ak );
       //radio.Send(ii2, payload2, al, requestACK);
        rf12_sendStart(  RF12_HDR_DST |ii2, &payload2, ak );// RF12_HDR_ACK | RF12_HDR_CTL |
       Serial.print (c3[ak-2]);
       if (c3[ak-2] > 48) {
            digitalWrite(led, HIGH);
            delay(20);
            digitalWrite(led, LOW);}
         else {
            digitalWrite(led, HIGH);
            delay(20);
            digitalWrite(led, LOW);
            delay(40);
            digitalWrite(led, HIGH);
            delay(20);
            digitalWrite(led, LOW);
            }
      }   
      if (c0[0] == 67 )  {     // c0[0] = "C"
        ii = (byte)c1[2] -48;  
        str[ii] = "";  str[ii] += c1; str[ii] += c2 ; str[ii] += c3;
        Serial.println (str[ii]);
        Serial.print (c3[4]);
       if (c3[4] > 48) {
            digitalWrite(led, HIGH);
            delay(20);
            digitalWrite(led, LOW);}
         else {
            digitalWrite(led, HIGH);
            delay(20);
            digitalWrite(led, LOW);
            delay(40);
            digitalWrite(led, HIGH);
            delay(20);
            digitalWrite(led, LOW);
            }
        //Serial.print(ii);    
      }        
      Serial.println();Serial.print(c0[0]);Serial.print(c1);Serial.print(c2[0]);Serial.println(c3);   
    }     
    if (rf12_recvDone() && rf12_crc == 0) {
        byte n = rf12_len;
        if (rf12_crc == 0) {
           // Serial.print("OK");
        } else {
            if (quiet)
                return;
            Serial.print(" ?");
            if (n > 20) // print at most 20 bytes if crc is wrong
                n = 20;
        }
        if (myNetGroup == 0) {
            #if SERIAL
            Serial.print("G ");
            Serial.print((int) rf12_grp);   
            //Serial.println(RF12_WANTS_ACK );
            //Serial.println(myNodeID & COLLECT);        
            Serial.println();
           //Serial.print("i=");
           //Serial.print((int) rf12_hdr & 31);
           // Serial.print(' ');
           #endif
           }
        for (byte i = 0; i < n-1; ++i) {
            //Serial.print(' ');
            Serial.print((char)rf12_data[i]);
        }
        Serial.println();
        /*Serial.println(RF12_WANTS_ACK );
        Serial.println( RF12_ACK_REPLY );*/         
        if (RF12_WANTS_ACK && (myNodeID & COLLECT) == 0) {
          byte id =rf12_data[2]-48;
         #if SERIAL
             Serial.print(id); Serial.println(ii); 
         #endif    
             if (ii == id) {              
               //Serial.println( rf12_hdr-32 );Serial.print(str[ii] );
               //RF12_HDR_ACK == 0;
               ak = str[ii].length()+1; char payload2[ak];
              String st= str[ii];
               st.toCharArray(payload2,ak );             
               rf12_sendStart(ii, &payload2, ak );
               #if SERIAL
               Serial.print(" ak=");Serial.print(ak);Serial.print( (char)payload2[ak-8] );
               #endif
               str[ii]="";ii=0;  }              
              else { 
               String ff="ABC";
               ak = ff.length()+1; char payload3[ak];
               ff.toCharArray(payload3,ak );
               rf12_sendStart(ii, &payload3, ak ); //
               //rf12_sendStart(RF12_ACK_REPLY, , 1);
               #if DEBUG
                Serial.print( rf12_hdr );
                Serial.print ("Led-soll" );Serial.println(ii);
               #endif
               }
               //Serial.println (str);
               
        }
    }
}
