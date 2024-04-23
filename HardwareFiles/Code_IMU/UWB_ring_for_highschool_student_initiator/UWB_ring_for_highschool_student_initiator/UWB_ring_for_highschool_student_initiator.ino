/*
 * Copyright (c) 2021 by Yifeng Cao <ycao361@gatech.edu>
 * Peer-peer protocol
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DWM1000-TWR.ino
 * 
 *  
 */

#include <SPI.h>
#include <math.h>
#include <DW1000.h>
#include <DW1000Constants.h>
#include "ringActionFunctions.h"
#include "genericFunctions.h"
#include "RangingContainer.h"
#include "Adafruit_LSM9DS1.h"
#include <SdFat.h>
#include <time.h>
#include<TimeLib.h>
#include<Wire.h>
// #include <Adafruit_ISM330DHCX.h>
// #include <Adafruit_LIS3MDL.h>
#include <LSM6DSOSensor.h>

TwoWire dev_i2c(&sercom3, SDA, SCL);  
LSM6DSOSensor AccGyr(&dev_i2c);

// Adafruit_LIS3MDL lis3mdl;
// Adafruit_ISM330DHCX ism330dhcx;

// PIN Macro
#define VBATPIN A2
#define LED_PIN 12
#define NOISE_PIN 13
#define GOOD_PIN 6
#define SILENCE_PIN 5
#define DEV_INDICATOR_PIN 13

//Timer for implementing timeouts
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

//Other Macros
#define USB_CONNECTION 0
#define DEBUG_PRINT 0


#if(OUR_UWB_FEATHER==1)
const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 19; // spi select pin
#endif

#if(AUS_UWB_FEATHER==1)
const uint8_t PIN_RST = 2; // reset pin
const uint8_t PIN_IRQ = 3; // irq pin
const uint8_t PIN_SS = 4; // spi select pin
#endif



// Global control variables
volatile boolean received = false;
volatile boolean error = false;
volatile int16_t numReceived = 0; // todo check int type
volatile boolean sendComplete = false;
volatile boolean RxTimeout = false;

byte beacon_msg[BEACON_MSG_LEN] = {0};
int dst_index = 2;
uint16_t current_seq = 0;
uint64_t prev_my_send_time[257] = {0};
uint64_t prev_my_recv_time[257] = {0};
uint64_t prev_other_send_time[257] = {0};
uint64_t prev_other_recv_time[257] = {0};
uint16_t prev_send_seq[257] = {0};
uint16_t prev_recv_seq[257] = {0};
int imu_start_idx = 0;

volatile uint8_t current_state = STATE_IDLE;
unsigned long silenced_at = 0;
long randNumber;
int currentSlots = 8;


STATES states;


// SD-card related variables
SdFat sd;
SdFile store_distance; //The root file system useful for deleting all files on the SDCard
char filename[14];
int filenum = 0;
int entries_in_file=0;
int SDChipSelect = 10;
int SDEnabled=0;


byte rx_buffer[1024];
// uint8_t myAcc[1000];
int32_t imu_buffer[IMU_BUFFER_SIZE][7] = {0};
int n_imu_samples = 0;

Ranging thisRange;

double currentTime;
double elapsedTime;

double globalTimeMs = 0;
double lastTimeMs = 0;
double currentTimeMs = 0;
boolean timerStart = false;
boolean start_new_receiver = 1;

//Time
boolean first_start = true;
double elapsed_time_us = 0;
double start_time_us = 0;
double send_info_start_time_us = 0;
double send_info_elapsed_time_us = 0;
int send_info_first_start = 1;
double global_time_us = 0;
double prev_imu_time_us = 0;
double elapsed_imu_time_us = 0;
int start_pattern_recognition = 1;
int n_cir_verifier = 0;

uint8_t raw_acc_mem[4063] = {0};
// uint8_t raw_acc_mem_verify[3][4063];
int cir_left_part = 10;
int cir_right_part = 50; // [a - cir_left, a+cir_right-1]
uint16_t rx_timeout_us = 0;
// uint16_t all_imu_data[2000][6];

void imu_setup();
void startTimer(int frequencyHz);
void setTimerFrequency(int frequencyHz);
void TC3_Handler();

typedef struct DeviceRespTs {
  int deviceID;
  uint64_t respRxTime;
};


void receiver(uint16_t rxtoval=0 ) {
  RxTimeout = false;
  received = false;
  DW1000.newReceive();
  set_customized_configs2();
  // we cannot don't need to restart the receiver manually
  DW1000.receivePermanently(false);
  DW1000.setRxTimeout(rxtoval);
  DW1000.startReceive();
}


void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(NOISE_PIN, OUTPUT);
  pinMode(GOOD_PIN, OUTPUT);
  pinMode(DEV_INDICATOR_PIN, OUTPUT);
  pinMode(SILENCE_PIN, INPUT_PULLUP);
  digitalWrite(GOOD_PIN, HIGH);
  analogReadResolution(10);
  // DEBUG monitoring
  Serial.begin(115200);
  while(!Serial)
  {
    delay(10);
    #if(USB_CONNECTION==0)
      break;
    #endif
  }
imu_setup();
Serial.print("Waiting...");
delay(5000);
Serial.print("Should see this...");



  randomSeed(analogRead(0));
  Serial.println(F("Peer-peer ranging protocol"));
  Serial.println("Free memory: ");
  Serial.println(freeMemory());
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  
  Serial.println(F("DW1000 initialized ..."));
  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  set_customized_configs2();
  DW1000.setDeviceAddress(6);
  DW1000.setNetworkId(10);
  // DW1000.enableMode(DW1000.MODE_UMPIRE_TRACKING);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));
  // DEBUG chip info and registers pretty printed
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.println(msg);
  Serial.print("Device mode: ");
  Serial.println(DW1000._deviceMode);
  uint8_t pac_size = DW1000._pacSize;
  Serial.print("Pac size: "); Serial.println(pac_size);
  uint8_t use_extended = DW1000._extendedFrameLength;
  Serial.print("Use extended frame: ");
  if (use_extended == 0x3){
    Serial.println(1);
  } else if (use_extended == 0){
    Serial.println(0);
  } else {
    Serial.println("?");
  }
  byte sfd_length[1];
  DW1000.readBytes(USR_SFD, 0, sfd_length, 1);
  Serial.print("SFD length: ");
  Serial.println(sfd_length[0]);
  byte channel_control[4];
  DW1000.readBytes(CHAN_CTRL, 0, channel_control, 4);
  Serial.print("Channel control: ");
  Serial.print(channel_control[0]);
  Serial.print(",");
  Serial.print(channel_control[1]);
  Serial.print(",");
  Serial.print(channel_control[2]);
  Serial.print(",");
  Serial.print(channel_control[3]);
  Serial.println("");

  byte tx_frame_ctrl[4];
  DW1000.readBytes(TX_FCTRL, 0, tx_frame_ctrl, 4);
  Serial.print("tx frame ctrl: ");
  Serial.print(tx_frame_ctrl[0]);
  Serial.print(",");
  Serial.print(tx_frame_ctrl[1]);
  Serial.print(",");
  Serial.print(tx_frame_ctrl[2]);
  Serial.print(",");
  Serial.print(tx_frame_ctrl[3]);
  Serial.println("");

  // byte sys_config[4];
  // DW1000.readBytes(SYS_CFG, 0, sys_config, LEN_SYS_CFG);
  // Serial.print("sys_config: ");
  // Serial.print(sys_config[0]);
  // Serial.print(",");
  // Serial.print(sys_config[1]);
  // Serial.print(",");
  // Serial.print(sys_config[2]);
  // Serial.print(",");
  // Serial.print(sys_config[3]);
  // Serial.println("");


  // attach callback for (successfully) received messages
  DW1000.attachReceivedHandler(handleReceived);
  DW1000.attachReceiveTimeoutHandler(handleRxTO);
  DW1000.attachReceiveFailedHandler(handleError);
  DW1000.attachErrorHandler(handleError);
  DW1000.attachSentHandler(handleSent);
  // start reception
  
  current_state = STATE_IDLE;

#if (INITIATOR == 1)
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(5000);
digitalWrite(DEV_INDICATOR_PIN, 0);
//delay(500);
#else
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(1000);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(200);
digitalWrite(DEV_INDICATOR_PIN, 0);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 1);
delay(500);
digitalWrite(DEV_INDICATOR_PIN, 0);
#endif
  // startTimer(80);
}


void handleSent() {
  // status change on sent success
  sendComplete = true;
  //Serial.println("Send complete");
}


void handleReceived() {
  // status change on reception success
  
  DW1000.getData(rx_buffer, DW1000.getDataLength());
  //Serial.println("Received something...");
  received = true;
  //show_packet(rx_buffer, DW1000.getDataLength());
}

void handleError() {
  Serial.println("Error!!!");
  error = true;
}

void handleRxTO() {
  RxTimeout = true;
  #if (DEBUG_PRINT==1)
  Serial.println("Rx Timeout");
  Serial.println("State: ");
  Serial.println(current_state);
  #endif
}



void test_tx_logic(){
  elapsed_time_us = get_elapsed_time_us(start_time_us);
  if (elapsed_time_us > 1e6) {
    current_seq = increment_seq(current_seq);
    Serial.print("Send a message: ");
    Serial.println(current_seq);
    send_beacon(1, current_seq, 0);
    start_time_us = get_time_us();
  }
  return;
}

void test_rx_logic(){
	if (first_start) {
    receiver(0);
    first_start = false;
    Serial.println("Start receiver!");
  }
  if (received) {
    uint16_t seq = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
    Serial.print("Receive a message: ");
    Serial.println(seq);
    received = false;
    receiver(0);
  }
  if (RxTimeout) {
    Serial.println("Rx Timeout!");
  }
}

void two_way_ranging_logic() {
  if(start_new_receiver){
    receiver(0);
  }
  start_new_receiver = 0;
  
  // Add one IMU data to buffer.
  elapsed_imu_time_us = get_elapsed_time_us(prev_imu_time_us);
   if (elapsed_imu_time_us > 1e6 / imu_rate) {
    
     add_imu_to_buffer(imu_buffer, n_imu_samples,MAX_IMU_SAMPLES_IN_BUFFER_TWRCIR);
     // Serial.print("n_imu_samples=");
     // Serial.println(n_imu_samples);
     prev_imu_time_us = get_time_us();
   }

  // UWB logic
  switch(current_state) {
    case STATE_IDLE: {
        //when every node initially starts, it has to wait a full loop in RECV state otherwise it will interrupt on-going ranging
        current_state = STATE_SEND;//STATE_RECEIVE;
        start_time_us = get_time_us();
        start_new_receiver = 1;
        break;
    }
    case STATE_RECEIVE:
			{
				elapsed_time_us = get_elapsed_time_us(start_time_us);
				if (elapsed_time_us > response_expect_timeout_us) {
					current_state = STATE_SEND;
          start_time_us = get_time_us();
					start_new_receiver = 0; // If the old receiver expires, start a new receiver. (The ring should avoid actively sending)
					break;
				} else {
					if (received == 1){
            // Serial.println("Receive a packet");
						receive_beacon();
            start_new_receiver = 0;
            current_state = STATE_SEND;
					} else {
            start_new_receiver = 0;
            // Serial.println("Continue to listen...");
						break;
					}
				}
				break;
			}
    case STATE_SEND:
    {
      Serial.println("Send a packet");
      current_seq = increment_seq(current_seq);
      initiator_send_beacon(dst_index, current_seq, prev_recv_seq[dst_index]);
      start_time_us = get_time_us();
      current_state = STATE_RECEIVE;
      start_new_receiver = 1;
      break;
    }
  }
}

void ring_logic_1_0(){
  if(start_new_receiver){
    receiver(rx_timeout_us);
  }
  start_new_receiver = 0;
  
  // Add one IMU data to buffer.
  elapsed_imu_time_us = get_elapsed_time_us(prev_imu_time_us);
  // if (elapsed_imu_time_us > 1e6 / imu_rate) {
    
    // add_imu_to_buffer(imu_buffer, n_imu_samples);
    // Serial.print("n_imu_samples=");
    // Serial.println(n_imu_samples);
    // prev_imu_time_us = get_time_us();
  // }

  // UWB logic
  switch(current_state) {
    case STATE_IDLE: 
    {
        //when every node initially starts, it has to wait a full loop in RECV state otherwise it will interrupt on-going ranging
        current_state = STATE_HS_REQUEST_EXPECTED;
        // rx_timeout_us = deca_timeout_us;
        rx_timeout_us = 0;
        start_time_us = get_time_us();
        start_new_receiver = 1;
        start_pattern_recognition = 0;
        n_imu_samples = 0;
        n_cir_verifier = 0;
        prev_imu_time_us = 0;
        imu_start_idx = 0;
        send_info_first_start = 1;
        break;
    }
    case STATE_HS_REQUEST_EXPECTED: 
    {
				if (received == 0){
          break;
        }
        received = 0;
        int status = receive_hs_request();
        if (status == 0) {
          start_new_receiver = 0;
          current_state = STATE_HS_SEND_REPLY;
        } else {
          start_new_receiver = 1;
          break;
        }
				break;
		}
    case STATE_HS_SEND_REPLY:
    {
      Serial.println("Send a reply packet");
      current_seq = increment_seq(current_seq);
      send_hs_reply(dst_index, current_seq, prev_recv_seq[dst_index]);
      start_time_us = get_time_us();
      current_state = STATE_HS_PCACK_EXPECTED;
      start_new_receiver = 1;
      rx_timeout_us = 0;
      break;
    }
    case STATE_HS_PCACK_EXPECTED:
    {
      elapsed_time_us = get_elapsed_time_us(start_time_us);
      if (elapsed_time_us > ONE_SECOND_TIMEOUT_US) {
        current_state = STATE_IDLE;
        start_time_us = get_time_us();
        rx_timeout_us = 0;
        start_new_receiver = 1; // If the old receiver expires, start a new receiver. (The ring should avoid actively sending)
        break;
      } 
      if (received == 0){
        start_new_receiver = 0;
        break;
      }
      received = 0;
      Serial.println("Receive something in STATE_HS_PCACK_EXPECTED");
      int status = receive_hs_pcack();
      if (status == 0) {
        current_state = STATE_HS_SEND_RINGACK;
        start_time_us = get_time_us();
        start_new_receiver = 0;
        Serial.println("Handshaking is successful. Wait for two way ranging.");
      } else {
        start_new_receiver = 1;
        break;
      }
      break;
    }
    case STATE_HS_SEND_RINGACK: {
      Serial.println("Send ROMG ACK, go to STATE_TWR_RECEIVE");
      current_seq = increment_seq(current_seq);
      send_hs_ringack(dst_index, current_seq, prev_recv_seq[dst_index]);
      start_time_us = get_time_us();
      current_state = STATE_TWR_RECEIVE;
      start_new_receiver = 1;
      rx_timeout_us = 0;
      break;
    }
    case STATE_TWR_RECEIVE: {
      elapsed_time_us = get_elapsed_time_us(start_time_us);
      if (elapsed_time_us > FIVE_SECOND_TIMEOUT_US) {
        current_state = STATE_IDLE;
        start_time_us = get_time_us();
        rx_timeout_us = 0;
        start_new_receiver = 1; 
        Serial.println("TWR receive timeout");
        break;
      } 
      if (received == 0){
        start_new_receiver = 0;
        break;
      }
      received = 0;
      int status = receive_twr_beacon();
      if (status == 0) {
        current_state = ((start_pattern_recognition == 1) ? STATE_PAT_SEND_REPLY:
          STATE_TWR_SEND);
        start_time_us = get_time_us();
        start_new_receiver = 0;
      } else {
        start_new_receiver = 1;
        break;
      }
      break;
    }
    case STATE_TWR_SEND: {
      Serial.println("Send a twr short beacon.");
      current_seq = increment_seq(current_seq);
      send_twr_beacon(dst_index, current_seq, prev_recv_seq[dst_index]);
      start_time_us = get_time_us();
      current_state = STATE_TWR_RECEIVE;
      start_new_receiver = 1;
      break;
    }
    case STATE_PAT_SEND_REPLY: {
      current_seq = increment_seq(current_seq);
      send_pat_reply(dst_index, current_seq, prev_recv_seq[dst_index]);
      start_time_us = get_time_us();
      current_state = STATE_PAT_SHORTBEACON_EXPECTED;
      start_new_receiver = 1;
      Serial.println("go to STATE_PAT_SHORTBEACON_EXPECTED");
      break;
    }
    case STATE_PAT_SHORTBEACON_EXPECTED:{
      // Collect IMU data and receive short beacon
      elapsed_imu_time_us = get_elapsed_time_us(prev_imu_time_us);
      if (elapsed_imu_time_us > 1e6 / imu_rate) {
    
        add_imu_to_buffer(imu_buffer, n_imu_samples, MAX_IMU_SAMPLES_IN_BUFFER_APP);
        prev_imu_time_us = get_time_us();
      }

      if (n_imu_samples >= MAX_IMU_SAMPLES && n_cir_verifier >= MAX_CIR_VERIFIER) {
        current_seq = increment_seq(current_seq);
        send_pat_info_start(dst_index, current_seq, prev_recv_seq[dst_index]);
        start_time_us = get_time_us();
        current_state = STATE_PAT_SEND_INFO_START;
        start_new_receiver = 0;
        Serial.println(n_cir_verifier);
        Serial.println("go to STATE_PAT_SEND_INFO_START");
        
        break;
      }
      if (elapsed_time_us > TEN_SECOND_TIMEOUT_US) {
        current_state = STATE_IDLE;
        start_time_us = get_time_us();
        rx_timeout_us = 0;
        start_new_receiver = 1; 
        break;
      } 
      if (received == 0){
        start_new_receiver = 0;
        break;
      }
      received = 0;
      int status = receive_pat_shortbeacon();
      if (status == 0) {
        Serial.println("WhatXXXXXXXX?");
        n_cir_verifier += 1;
      } 
      start_new_receiver = 1;
      break;
    }
    case STATE_PAT_SEND_INFO_START: {
      current_seq = increment_seq(current_seq);
      send_pat_info_start(dst_index, current_seq, prev_recv_seq[dst_index]);
      start_time_us = get_time_us();
      current_state = STATE_PAT_INFO_ACK_EXPECTED;
      start_new_receiver = 1;
      break;
    }
    case STATE_PAT_INFO_ACK_EXPECTED: {
      elapsed_time_us = get_elapsed_time_us(start_time_us);
      if (elapsed_time_us > ONE_SECOND_TIMEOUT_US) {
        current_state = STATE_IDLE;
        start_time_us = get_time_us();
        rx_timeout_us = 0;
        start_new_receiver = 1; 
        break;
      } 
      if (received == 0){
        start_new_receiver = 0;
        break;
      }
      received = 0;
      int status = receive_pat_info_ack();
      if (status == 0) {
        current_state = STATE_PAT_SEND_INFO;
        start_time_us = get_time_us();
        start_new_receiver = 0;
        Serial.println("Go to STATE_PAT_SEND_INFO");
      } else {
        start_new_receiver = 1;
      }
      break;
    }
    case STATE_PAT_SEND_INFO:{
      current_seq = increment_seq(current_seq);
      send_pat_info(dst_index, current_seq, prev_recv_seq[dst_index], imu_start_idx);
      start_time_us = get_time_us();
      current_state = STATE_PAT_INFO_IMU_ACK_EXPECTED;
      start_new_receiver = 1;
      if (send_info_first_start == 1){
        send_info_first_start = 0;
        send_info_start_time_us = get_time_us();
      }
      Serial.print("Send info: ");
      Serial.println(current_seq);
      break;
    }
    case STATE_PAT_INFO_IMU_ACK_EXPECTED:{
      send_info_elapsed_time_us = get_elapsed_time_us(send_info_start_time_us);
      if (send_info_elapsed_time_us > FIVE_SECOND_TIMEOUT_US) {
        current_state = STATE_IDLE;
        start_time_us = get_time_us();
        rx_timeout_us = 0;
        start_new_receiver = 1; 
        break;
      } 
      elapsed_time_us = get_elapsed_time_us(start_time_us);
      if (elapsed_time_us > STANDARD_TIMEOUT_US) {
        current_state = STATE_PAT_SEND_INFO;
        start_time_us = get_time_us();
        rx_timeout_us = 0;
        start_new_receiver = 0; 
        break;
      } 
      if (received == 0){
        start_new_receiver = 0;
        break;
      }
      received = 0;
      int status = receive_pat_info_imu_ack();
      if (status == RET_RECEIVE_IMU_ACK) {
        current_state = STATE_PAT_SEND_INFO;
        start_new_receiver = 0;
        imu_start_idx += MAX_IMU_EACH_PACKET_TYPICAL;
        if (imu_start_idx >= MAX_IMU_SAMPLES) {
          current_state = STATE_ALL_COMPLETE;
          Serial.println("All complete!!");
          break;
        }
      } else if (status == RET_RECEIVE_WRONG_ACK){
        current_state = STATE_PAT_SEND_INFO;
        start_new_receiver = 0;
      } else {
        start_new_receiver = 1;
      }
      break;
    }
    case STATE_ALL_COMPLETE:{
      break;
    }
      
  }
}

void loop() {
  two_way_ranging_logic();
  // ring_logic_1_0();
}





void show_packet(byte packet[], int num) {
  #if (DEBUG_PRINT==1)
  for (int i=0;i<num;i++) {
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  #endif
  
}

//Timer Functions
//Timer functions.
void setTimerFrequency(int frequencyHz) {
  int compareValue = (CPU_HZ / (TIMER_PRESCALER_DIV * frequencyHz)) - 1;
  
  TcCount16* TC = (TcCount16*) TC3;
  // Make sure the count is in a proportional position to where it was
  // to prevent any jitter or disconnect when changing the compare value.
  TC->COUNT.reg = map(TC->COUNT.reg, 0, TC->CC[0].reg, 0, compareValue);
  TC->CC[0].reg = compareValue;
  //Serial.println(TC->COUNT.reg);
  //Serial.println(TC->CC[0].reg);
  while (TC->STATUS.bit.SYNCBUSY == 1);
}

void startTimer(int frequencyHz) {
  
  REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3) ;
  
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync

  TcCount16* TC = (TcCount16*) TC3;
  
  TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use the 16-bit timer
  TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Use match mode so that the timer counter resets when the count matches the compare register
  TC->CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  // Set prescaler to 1024
  TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024;
  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync

  setTimerFrequency(frequencyHz);

  // Enable the compare interrupt
  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.MC0 = 1;

  
    NVIC_SetPriority(TC3_IRQn, 3);
    NVIC_EnableIRQ(TC3_IRQn);  
    

  TC->CTRLA.reg |= TC_CTRLA_ENABLE;

  while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync
}

void TC3_Handler() 
{
  TcCount16* TC = (TcCount16*) TC3;
  // If this interrupt is due to the compare register matching the timer count
  // we toggle the LED.
  if (TC->INTFLAG.bit.MC0 == 1) {
    TC->INTFLAG.bit.MC0 = 1;
    
    //  collect_imu_data();

  }
}

void imu_setup(){
  dev_i2c.begin();
  
  // Initialize I2C bus.
  LSM6DSOSensor AccGyr(&dev_i2c);
  AccGyr.begin();

  AccGyr.Set_X_ODR(416); // Acc 416Hz
  AccGyr.Set_X_FS(4); // Acc range: 4g  
  AccGyr.Set_G_ODR(416); // Gyro 416Hz
  AccGyr.Set_G_FS(500); // Gyro range: 500 dps
  AccGyr.Enable_X(); 
  AccGyr.Enable_G();
}

//Utility functions

float getVoltage()
{
  
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  return measuredvbat;
}

// void send_beacon(uint8_t dst, uint16_t seq_no, uint16_t ack_no){
// 	beacon_msg[MSG_TYPE_IDX] = BEACON_TYPE;
// 	beacon_msg[SRC_IDX] = my_index;
// 	beacon_msg[DST_IDX] = dst;
// 	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
// 	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
// 	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
// 	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
// 	prev_send_seq[dst] = seq_no;

// 	// Set prevRecvTime field in packet.
// 	set_timestamp_to_packet_u64(&beacon_msg[PREV_RX_TS_IDX], prev_my_recv_time[dst]);

//   // Set prevSendTime field in packet.
//   DW1000.newTransmit();
//   DW1000Time txTime;
//   DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
//   txTime = DW1000.setDelay(deltaTime);
//   prev_my_send_time[dst]= txTime.getTimestamp();
// 	set_timestamp_to_packet_u64(&beacon_msg[CURRENT_TX_TS_IDX], prev_my_send_time[dst]);
// 	generic_send(beacon_msg, sizeof(beacon_msg));

//   while(!sendComplete);
//   sendComplete = false;
// }



#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__



int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}
