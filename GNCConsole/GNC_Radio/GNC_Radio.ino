#include <SPI.h>
#include <RH_RF95.h>
#include <RHReliableDatagram.h>

int x[] = {1, 2, 3, 4, 5};
char Sermode;
String test;
bool state = false;


//Define pins
#define LED 0
#define RFM95_CS 16
#define RFM95_RST 5
#define RFM95_INT 4
#define RF95_FREQ 915.0

//Define addresses
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

RH_RF95 rf95(RFM95_CS, RFM95_INT);

RHReliableDatagram manager(rf95, SERVER_ADDRESS);

void setup(){
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(LED, HIGH);
  Serial.begin(115200);
  //Serial.println("Arduno LoRa RX Test!");

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  digitalWrite(LED, LOW);
  //Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    //Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

void loop()
{
  Serial.print('WAITING');
  if (manager.available())
  Serial.println("MessageAvailable");
  {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (manager.recvfromAck(buf, &len))
    {
      Serial.println("got request from : 0x");
      digitalWrite(LED, HIGH);
      //RH_RF95::printBuffer("Received: ", buf, len);
      //Serial.print("Got: ");
      test = (char*)buf;
      Serial.println(test);
      //Serial.println((char*)buf);
      // Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);

      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      //Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      //Serial.println("Receive failed");
    }
  }
  if(Serial.available()){

      Sermode = Serial.read();
      if(Sermode == 'g'){
        Serial.println(test);
        }
      }


}
