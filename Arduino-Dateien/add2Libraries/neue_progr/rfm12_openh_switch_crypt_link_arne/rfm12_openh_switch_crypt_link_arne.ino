// Simple serial pass through program
// It initializes the RFM12B radio with optional encryption and passes through any valid messages to the serial port
// felix@lowpowerlab.com
// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#include <RFM12B.h>
#define NODEID           1  //network ID used for this unit
#define NETWORKID       99  //the network ID we are on
#define SERIAL_BAUD 9600
#include <OneWire.h>
#include <RCSwitch.h> //Lib zur Steuerung von Elro und Homeeasysteckdosen
//#include <DallasTemperature.h>
//#define ONE_WIRE_BUS 8
//OneWire oneWire(ONE_WIRE_BUS); 
//DallasTemperature sensors(&oneWire);
RCSwitch mySwitch = RCSwitch();
uint8_t KEY[] = "ABCDABCDABCDABCD";
RFM12B radio;
void setup() {
    //sensors.begin();
    mySwitch.enableReceive(1); // interrupt 1 == arduino pin 3
    mySwitch.enableTransmit(9);
    mySwitch.setPulseLength(325);
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  radio.Encrypt(KEY);      //comment this out to disable encryption
  Serial.begin(SERIAL_BAUD);
  Serial.println("Listening...");
}
char c0[3], c1[6], c2[3], c3[6];
void loop() {  
  if (mySwitch.available()) {  
    Serial.println( mySwitch.getReceivedValue() );  
    long value = mySwitch.getReceivedValue();      
      //Serial.print( mySwitch.getReceivedValue() );         
    if (value == 327957) {
      Serial.print("Received ");
      Serial.println( mySwitch.getReceivedValue() );
      //Serial.print("Unknown encoding");
      mySwitch.switchOn("11110", "10000");
      mySwitch.switchOn("11110", "10000");
    } 
      if (value == 327956) {
      mySwitch.switchOff("11110", "10000");
      mySwitch.switchOff("11110", "10000");
      Serial.print("Received ");
      Serial.println( mySwitch.getReceivedValue() );
    }
   //sensors.requestTemperatures();
   //Serial.print("i=5;t=t;v=");
   //Serial.println(sensors.getTempCByIndex(0)); 
   mySwitch.resetAvailable(); 
   } 
   
   if (Serial.available() > 0) {
     Serial.readBytes(c0, 1); Serial.readBytes(c1, 5);Serial.readBytes(c2, 1); Serial.readBytes(c3, 5); 
     // einfache Lösung um seriellen Datenstrom in Variablen zu speichern
     if (c0[0] == 65 ) {mySwitch.switchOn(c1, c3); }  // c0[0] = "A"         
     if (c0[0] == 66 ) {mySwitch.switchOff(c1, c3); } // c0[0] = "B"             
     Serial.println();Serial.print(c0[0]);Serial.print(c1);Serial.print(c2[0]);Serial.println(c3);   
    }    

 if (radio.ReceiveComplete())
  {
    if (radio.CRCPass())
    {
      Serial.print("i=");Serial.print(radio.GetSender());Serial.print(";");
      for (byte i = 0; i < *radio.DataLen; i++) //can also use radio.GetDataLen() if you don't like pointers
        Serial.print((char)radio.Data[i]);
      if (radio.ACKRequested())
      {
        radio.SendACK();
        Serial.println();
        //Serial.print(" - ACK sent");
      }
    }
    else
      Serial.print("BAD-CRC");    
      Serial.println();   
  }
}


