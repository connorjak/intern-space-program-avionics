// rf95_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_RF95 driver to control a RF95 radio.
// It is designed to work with the other example rf95_reliable_datagram_client
// Tested with Anarduino MiniWirelessLoRa, Rocket Scream Mini Ultra Pro with the RFM95W
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3
String recievedMessage;
char command;
String msg;
// Singleton instance of the radio driver
RH_RF95 driver(RFM95_CS, RFM95_INT);
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W
// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);
// Need this on Arduino Zero with SerialUSB port (eg RocketScream Mini Ultra Pro)
//#define Serial SerialUSB
void setup()
{
  // Rocket Scream Mini Ultra Pro with the RFM95W only:
  // Ensure serial flash is not interfering with radio communication on SPI bus
//  pinMode(4, OUTPUT);
//  digitalWrite(4, HIGH);
  Serial.begin(9600);
  if (!manager.init())
    Serial.println("init failed");
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
//  driver.setTxPower(23, false);
  // If you are using Modtronix inAir4 or inAir9,or any other module which uses the
  // transmitter RFO pins and not the PA_BOOST pins
  // then you can configure the power transmitter power for -1 to 14 dBm and with useRFO true.
  // Failure to do that will result in extremely low transmit powers.
//  driver.setTxPower(14, true);
  // You can optionally require this module to wait until Channel Activity
  // Detection shows no activity on the channel before transmitting by setting
  // the CAD timeout to non-zero:
//  driver.setCADTimeout(10000);
Serial.println("Thunderbirds are go!");
}
uint8_t data[] = "And hello back to you";
uint8_t testmsg[] = "Response"
// Dont put this on the stack:
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
void loop()
{
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      /*Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);*/
      // Send a reply back to the originator client
      recievedMessage = (char*)buf;

    }
  }
  if(Serial.available()){
    command = Serial.read();
    delay(2);
    if(command == 'g'){
      Serial.println(recievedMessage);
    }else if(command == 's'){
      while (Serial.available()) {
        char c = Serial.read();  //gets one byte from serial buffer
        msg += c; //makes the String readString
        delay(2);  //slow looping to allow buffer to fill with next character
      }
      msg.toCharArray(data, sizeof(msg));
      if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS)){

      }
      else{
      }
    }
    else if (command == 't'){
      if (manager.sendtoWait(testmsg, sizeof(data), SERVER_ADDRESS)){

      }
      else{
      }
    }
  }
}
