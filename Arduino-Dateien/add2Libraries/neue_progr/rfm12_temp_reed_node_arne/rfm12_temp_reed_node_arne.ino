// Simple RFM12B sender program, with ACK and optional encryption
// It initializes the RFM12B radio with optional encryption and passes through any valid messages to the serial port
// felix@lowpowerlab.com

#include <RFM12B.h>
#include <avr/sleep.h>
#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 9
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// You will need to initialize the radio by telling it what ID it has and what network it's on
// The NodeID takes values from 1-127, 0 is reserved for sending broadcast messages (send to all nodes)
// The Network ID takes values from 0-255
// By default the SPI-SS line used is D10 on Atmega328. You can change it by calling .SetCS(pin) where pin can be {8,9,10}
#define NODEID        3  //network ID used for this unit
#define NETWORKID    99  //the network ID we are on
#define GATEWAYID     1  //the node ID we're sending to
#define ACK_TIME     50  // # of ms to wait for an ack
#define SERIAL_BAUD  9600

//encryption is OPTIONAL
//to enable encryption you will need to:
// - provide a 16-byte encryption KEY (same on all nodes that talk encrypted)
// - to call .Encrypt(KEY) to start encrypting
// - to stop encrypting call .Encrypt(NULL)
uint8_t KEY[] = "ABCDABCDABCDABCD";

int interPacketDelay = 10000; //wait this many ms between sending packets
char input = 0;

// Need an instance of the Radio Module
RFM12B radio;
byte sendSize=12;
bool requestACK=false;
float temp;
int reedp = 8;//pin 8 für reedkontakt als Eingang
void setup()
{
  pinMode(reedp, INPUT);
  sensors.begin();
  Serial.begin(SERIAL_BAUD);
  radio.Initialize(NODEID, RF12_868MHZ, NETWORKID);
  radio.Encrypt(KEY);
  radio.Sleep(); //sleep right away to save power
  Serial.println("Transmitting...\n\n");
}
void loop()
{ 
  sensors.requestTemperatures();
  temp =  sensors.getTempCByIndex(0);
  char payload2[7];
  dtostrf(temp, 6,2, payload2);
  Serial.print("Sending[");
  Serial.print(sendSize);
  Serial.print("]:"); 
  requestACK = true ;//!(sendSize % 3); //request ACK every 3rd xmission  
  String aa = "t=t;v="; String cc;  cc =  aa + payload2;  
  Serial.println(cc);
  char payload[cc.length()+1];  cc.toCharArray(payload, cc.length()+1);
 
  radio.Wakeup();
  sendSize=12;
  radio.Send(GATEWAYID, payload, sendSize, requestACK);
  delay(100);  
  //char payload3[8]; //cc =  aa + payload3;
  String dd ="t=s;v=1";
  if (digitalRead(8)) dd="t=s;v=1";
  else dd = "t=s;v=0";  
  sendSize=7;
  char payload3[dd.length()+1];  dd.toCharArray(payload3, dd.length()+1);
  radio.Send(GATEWAYID, payload3, sendSize, requestACK);
  
  if (requestACK)
  {
    Serial.print(" - waiting for ACK...");
    if (waitForAck()) Serial.print("ok!");
    else Serial.print("nothing...");
  }
  radio.Sleep();
  
  //sendSize = (sendSize + 1) % 88;
  Serial.println();
  delay(interPacketDelay);
}

// wait a few milliseconds for proper ACK, return true if received
static bool waitForAck() {
  long now = millis();
  while (millis() - now <= ACK_TIME)
    if (radio.ACKReceived(GATEWAYID))
      return true;
  return false;
}
