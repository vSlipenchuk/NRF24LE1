/*
TMRh20 2014

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/** General Data Transfer Rate Test
 * This example demonstrates basic data transfer functionality with the 
 updated library. This example will display the transfer rates acheived using
 the slower form of high-speed transfer using blocking-writes.
 */


#include <SPI.h>
#include "RF24.h"
#include "printf.h"

/*************  USER Configuration *****************************/
                                          // Hardware configuration
RF24 radio(40,41);                        // Set up nRF24L01 radio on SPI bus plus pins 8 & 9


/***************************************************************/

//const uint64_t pipes[2] = { 0xABCDABCD71LL, 0x544d52687CLL };   // Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL,0xF0F0F0F0D2LL};
//const uint64_t pipes[2] = { 0xF0F0F0F0D2LL,0xF0F0F0F0E1LL};


byte data[32];                           //Data buffer for testing data transfer speeds

unsigned long counter; // rxTimer;          //Counter and timer for keeping track transfer info
//unsigned long startTime, stopTime;  
//bool TX=1,RX=0,role=0;

void write_back(byte d0,byte d1,byte d2,byte d3) {
  data[0]=d0; data[1]=d1; data[2]=d2; data[3]=d3; data[4]=d0+d1+d2+d3;
  radio.stopListening();
  radio.write(&data,32);
  radio.startListening();
}


void setup(void) {

  // The UNO Adapter uses pin 10 as Vcc
  pinMode(10,OUTPUT);
  digitalWrite(10,HIGH);
  delay(500);
  
  Serial.begin(57600);
  printf_begin();

  radio.begin();                           // Setup and configure rf radio
  radio.setChannel(100);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  radio.setAutoAck(false);                     // Ensure autoACK is enabled
  radio.setRetries(2,15);                   // Optionally, increase the delay between retries & # of retries
  radio.setCRCLength(RF24_CRC_16); 
  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);
  
  radio.startListening();                 // Start listening
  radio.printDetails();                   // Dump the configuration of the rf unit for debugging
  
  printf("\n\rRF24/examples/Transfer Rates/\n\r");
  printf("*** PRESS 'T' to begin transmitting to the other node\n\r");
  
  //randomSeed(analogRead(0));              //Seed for random number generation
  
  radio.powerUp();                        //Power up the radio

   write_back(1,10,1,1); // blink once
      
}


unsigned char cntr=0;


void loop(void){
  
while(radio.available()){        // print all we read
      int i,len=32;
      radio.read(&data,32);
      for(i=0;i<len;i++) printf("%d ",data[i]);
      printf("   pack len=%d pack=%d\n",len,counter); counter++;
       //  write_back(1,10,1,1); // blink
      }
  delay(600);
  write_back(1,10,1,cntr);   printf("write cntr=%d\n",cntr);
  cntr++;
}
