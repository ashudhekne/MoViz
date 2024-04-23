#ifndef RING_ACTION_FUNCTIONS_H
#define RING_ACTION_FUNCTIONS_H
#include <SPI.h>
#include <math.h>
#include <DW1000.h>
#include <DW1000Constants.h>

#include "genericFunctions.h"
#include "RangingContainer.h"
#include "Adafruit_LSM9DS1.h"
#include <SdFat.h>
#include <time.h>
#include<TimeLib.h>
#include<Wire.h>
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>

extern byte beacon_msg[BEACON_MSG_LEN];
extern int dst_index;
extern uint16_t current_seq;
extern uint64_t prev_my_send_time[257];
extern uint64_t prev_my_recv_time[257];
extern uint64_t prev_other_send_time[257];
extern uint64_t prev_other_recv_time[257];
extern uint16_t prev_send_seq[257];
extern uint16_t prev_recv_seq[257];
extern int cir_left_part;
extern int cir_right_part;
extern uint8_t raw_acc_mem[4063];
extern byte rx_buffer[1024];
extern Ranging thisRange;
extern int start_pattern_recognition;
extern int n_cir_verifier;

extern int32_t imu_buffer[IMU_BUFFER_SIZE][7];
extern int n_imu_samples;

extern volatile boolean sendComplete;

int send_beacon(uint8_t dst, uint16_t seq_no, uint16_t ack_no){
	beacon_msg[MSG_TYPE_IDX] = BEACON_TYPE;
	beacon_msg[SRC_IDX] = my_index;
	beacon_msg[DST_IDX] = dst;
	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
	prev_send_seq[dst] = seq_no;

    // embed imus.
    // Serial.print("n_imu_samples: ");
    // Serial.println(n_imu_samples);
    embed_imus(&beacon_msg[SKETCH_N_IMU_IDX], imu_buffer, n_imu_samples);
    n_imu_samples = 0;
	// Set prevRecvTime field in packet.
	set_timestamp_to_packet_u64(&beacon_msg[PREV_RX_TS_IDX], prev_my_recv_time[dst]);

    // Set prevSendTime field in packet.
    DW1000.newTransmit();
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
    txTime = DW1000.setDelay(deltaTime);
    prev_my_send_time[dst]= txTime.getTimestamp();
	set_timestamp_to_packet_u64(&beacon_msg[CURRENT_TX_TS_IDX], prev_my_send_time[dst]);
	generic_send(beacon_msg, sizeof(beacon_msg));

    while(!sendComplete);
    sendComplete = false;
}

int receive_beacon(){
    // dst id check; [seq check (not implemented yet)];
    // Serial.println("Receive a packet");
    char output_msg[1024];
    int output_msg_len = 0;
	uint8_t src_id = rx_buffer[SRC_IDX];
	uint8_t dst_id = rx_buffer[DST_IDX];
	uint16_t seq_no = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
	uint16_t ack_no = rx_buffer[ACK_IDX] + ((rx_buffer[ACK_IDX+1]) << 8);
	if(dst_id != my_index){
    output_msg_len += sprintf(&output_msg[output_msg_len],
    "Destination is not me. Dst: [%d]", rx_buffer[DST_IDX]);
		Serial.println(output_msg);
		return 1;
	}

    uint64_t poll_tx_ts = prev_other_send_time[src_id];
    uint64_t poll_rx_ts = prev_my_recv_time[src_id];
    uint64_t resp_tx_ts = prev_my_send_time[src_id];
    uint64_t resp_rx_ts = get_timestamp_from_packet_u64(&rx_buffer[PREV_RX_TS_IDX]);
    uint64_t final_tx_ts = get_timestamp_from_packet_u64(&rx_buffer[CURRENT_TX_TS_IDX]);
    uint64_t final_rx_ts = get_rx_timestamp_u64();
    thisRange.PollTxTime =  DW1000Time((int64_t) poll_tx_ts);
    thisRange.PollRxTime =  DW1000Time((int64_t) poll_rx_ts);
    thisRange.RespTxTime =  DW1000Time((int64_t) resp_tx_ts);
    thisRange.RespRxTime =  DW1000Time((int64_t) resp_rx_ts);
    thisRange.FinalTxTime =  DW1000Time((int64_t) final_tx_ts);
    thisRange.FinalRxTime =  DW1000Time((int64_t) final_rx_ts);
  

	prev_other_recv_time[src_id] = resp_rx_ts;
	prev_other_send_time[src_id] = final_tx_ts;
	prev_my_recv_time[src_id] = final_rx_ts;
	prev_recv_seq[src_id] = seq_no;

	int distance_int = thisRange.calculateRange();
    /* Get receive diagnostics. */
    uint16_t max_noise = get_max_noise();
    uint16_t std_noise = get_std_noise();
    uint16_t max_growth_cir = get_max_growth_cir();
    uint16_t rx_pream_count = get_rx_pream_count();
    double rq = DW1000.getReceiveQuality();
    double fp_power = DW1000.getFirstPathPower();
    double rx_power = DW1000.getReceivePower();

    double current_time = get_time_ms();

    output_msg_len += sprintf(&output_msg[output_msg_len], 
        "%d,%d,%d,%d,%d,%.2f,", 
        OUTPUT_TWR_CIR_TYPE, my_index, src_id, seq_no, distance_int, current_time);
  int first_path_index = (int) get_first_peak();

  // An example: cir_start=720, cir_end=760, you should collect [720,759]
  int cir_start = first_path_index - cir_left_part;
  int cir_end = first_path_index + cir_right_part; // Make sure end is not included

  int n_cir_taps = cir_end - cir_start;
  int raw_acc_mem_byte_len = 4 * (cir_end - cir_start);

  /* Print decimal cir*/
//   int16_t RealData = 0;
//   int16_t ImaginaryData = 0;
//   getAccMemYifeng(raw_acc_mem, cir_start, cir_end);//myAcc will contain 16 bit real imaginary pairs
//   for (int i = 0 ; i< n_cir_taps; i++){
//     RealData = (((uint16_t)(raw_acc_mem[(i * 4) + 1])) << 8) | raw_acc_mem[(i * 4) + 0];
//     ImaginaryData = (((uint16_t)(raw_acc_mem[(i * 4) + 3])) << 8) | raw_acc_mem[(i * 4) + 2];
//     output_msg_len += sprintf(&output_msg[output_msg_len], "%d,%d,", RealData, ImaginaryData);
//   }

  Serial.println(output_msg);


  // Serial.print("Receive a packet from ");
  //   Serial.print(src_id);
  //   Serial.print(", seq = ");
  //   Serial.print(seq_no);
  //   Serial.print(", all ts = ");
  //   print_uint64(poll_tx_ts);
  //   Serial.print(", ");
  //   print_uint64(poll_rx_ts);
  //   Serial.print(", ");
  //   print_uint64(resp_tx_ts);
  //   Serial.print(", ");
  //   print_uint64(resp_rx_ts);
  //   Serial.print(", ");
  //   print_uint64(final_tx_ts);
  //   Serial.print(", ");
  //   print_uint64(final_rx_ts);
  //   Serial.print(", ");
  //   Serial.println("");
  return 0;
}

int receive_hs_request(){
    int twr_status_n;
    uint8_t msg_type = rx_buffer[MSG_TYPE_IDX];
	uint8_t src_id = rx_buffer[SRC_IDX];
	uint8_t dst_id = rx_buffer[DST_IDX];
	uint16_t seq_no = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
	uint16_t ack_no = rx_buffer[ACK_IDX] + ((rx_buffer[ACK_IDX+1]) << 8);
    if (msg_type != HS_REQUEST_TYPE){
        Serial.print("[receive_hs_request()] Wrong type: ");
        Serial.println(msg_type, HEX);
        return 1;
    }
	if(dst_id != my_index){
        Serial.print("[receive_hs_request()] Wrong dst: ");
        Serial.println(dst_id);
		return 1;
	}
    return 0;
}

int send_hs_reply(uint8 dst, uint16 seq_no, uint16 ack_no){
    memset(beacon_msg, 0, sizeof(beacon_msg));
	beacon_msg[MSG_TYPE_IDX] = HS_REPLY_TYPE;
	beacon_msg[SRC_IDX] = my_index;
	beacon_msg[DST_IDX] = dst;
	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
	prev_send_seq[dst] = seq_no;
    

    for (int i = 0; i < SESSION_KEY_LEN; i++){
        beacon_msg[i + SESSION_KEY_IDX] = my_session_key[i];
    }
    DW1000.newTransmit();
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
	generic_send(beacon_msg, sizeof(beacon_msg));
    Serial.println(beacon_msg[0]);
    while(!sendComplete);
    sendComplete = false;
    return 0;
}

int receive_hs_pcack(){
	int twr_status_n;
    uint8_t msg_type = rx_buffer[MSG_TYPE_IDX];
	uint8_t src_id = rx_buffer[SRC_IDX];
	uint8_t dst_id = rx_buffer[DST_IDX];
	uint16_t seq_no = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
	uint16_t ack_no = rx_buffer[ACK_IDX] + ((rx_buffer[ACK_IDX+1]) << 8);
    if (msg_type != HS_PCACK_TYPE){
        Serial.print("[receive_hs_pcack()] Wrong type: ");
        Serial.println(msg_type, HEX);
        return 1;
    }
	if(dst_id != my_index){
        Serial.print("[receive_hs_pcack()] Wrong dst: ");
        Serial.println(dst_id);
		return 1;
	}
	for (int i = 0; i < SESSION_KEY_LEN; i++) {
        if (my_session_key[i] != rx_buffer[SESSION_KEY_IDX+i]) {
            Serial.print("[receive_hs_pcack()] Wrong session key: expect ");
            Serial.print(my_session_key[i]);
            Serial.print(", get ");
            Serial.println(rx_buffer[SESSION_KEY_IDX+i]);
            return 1;
        }
	}
    return 0;
}

int send_hs_ringack(uint8 dst, uint16 seq_no, uint16 ack_no){
    memset(beacon_msg, 0, sizeof(beacon_msg));
	beacon_msg[MSG_TYPE_IDX] = HS_RINGACK_TYPE;
	beacon_msg[SRC_IDX] = my_index;
	beacon_msg[DST_IDX] = dst;
	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
	prev_send_seq[dst] = seq_no;
	beacon_msg[HS_RINGACK_STATUS_IDX] = 0;

    DW1000.newTransmit();
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
	generic_send(beacon_msg, sizeof(beacon_msg));
    while(!sendComplete);
    sendComplete = false;
    return 0;
}

int send_twr_beacon(uint8_t dst, uint16_t seq_no, uint16_t ack_no){
	beacon_msg[MSG_TYPE_IDX] = TWR_BEACON_TYPE;
	beacon_msg[SRC_IDX] = my_index;
	beacon_msg[DST_IDX] = dst;
	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
	prev_send_seq[dst] = seq_no;

    // embed imus.
    // Serial.print("n_imu_samples: ");
    // Serial.println(n_imu_samples);
    // embed_imus(&beacon_msg[N_IMU_IDX], imu_buffer, n_imu_samples);
    
	// Set prevRecvTime field in packet.
	set_timestamp_to_packet_u64(&beacon_msg[PREV_RX_TS_IDX], prev_my_recv_time[dst]);

    // Set prevSendTime field in packet.
    DW1000.newTransmit();
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
    txTime = DW1000.setDelay(deltaTime);
    prev_my_send_time[dst]= txTime.getTimestamp();
	set_timestamp_to_packet_u64(&beacon_msg[CURRENT_TX_TS_IDX], prev_my_send_time[dst]);
	generic_send(beacon_msg, sizeof(beacon_msg));

    while(!sendComplete);
    sendComplete = false;
}

int receive_twr_beacon(){
    char output_msg[2048];
    int output_msg_len = 0;
	uint8_t msg_type = rx_buffer[MSG_TYPE_IDX];
	uint8_t src_id = rx_buffer[SRC_IDX];
	uint8_t dst_id = rx_buffer[DST_IDX];
	uint16_t seq_no = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
	uint16_t ack_no = rx_buffer[ACK_IDX] + ((rx_buffer[ACK_IDX+1]) << 8);
    if (msg_type != TWR_BEACON_TYPE && msg_type != PAT_REQUEST_TYPE){
        Serial.print("[receive_twr_beacon()] Wrong type: ");
        Serial.println(msg_type, HEX);
        return ERROR_GENERAL;
    }
	if(dst_id != my_index){
        Serial.print("[receive_twr_beacon()] Wrong dst: ");
        Serial.println(dst_id);
		return ERROR_GENERAL;
	}
    
    uint64_t poll_tx_ts = prev_other_send_time[src_id];
    uint64_t poll_rx_ts = prev_my_recv_time[src_id];
    uint64_t resp_tx_ts = prev_my_send_time[src_id];
    uint64_t resp_rx_ts = get_timestamp_from_packet_u64(&rx_buffer[PREV_RX_TS_IDX]);
    uint64_t final_tx_ts = get_timestamp_from_packet_u64(&rx_buffer[CURRENT_TX_TS_IDX]);
    uint64_t final_rx_ts = get_rx_timestamp_u64();
    thisRange.PollTxTime =  DW1000Time((int64_t) poll_tx_ts);
    thisRange.PollRxTime =  DW1000Time((int64_t) poll_rx_ts);
    thisRange.RespTxTime =  DW1000Time((int64_t) resp_tx_ts);
    thisRange.RespRxTime =  DW1000Time((int64_t) resp_rx_ts);
    thisRange.FinalTxTime =  DW1000Time((int64_t) final_tx_ts);
    thisRange.FinalRxTime =  DW1000Time((int64_t) final_rx_ts);
  

	prev_other_recv_time[src_id] = resp_rx_ts;
	prev_other_send_time[src_id] = final_tx_ts;
	prev_my_recv_time[src_id] = final_rx_ts;
	prev_recv_seq[src_id] = seq_no;

	int distance_int = thisRange.calculateRange();
    /* Get receive diagnostics. */
    uint16_t max_noise = get_max_noise();
    uint16_t std_noise = get_std_noise();
    uint16_t max_growth_cir = get_max_growth_cir();
    uint16_t rx_pream_count = get_rx_pream_count();
    double rq = DW1000.getReceiveQuality();
    double fp_power = DW1000.getFirstPathPower();
    double rx_power = DW1000.getReceivePower();

    double current_time = get_time_ms();

    output_msg_len += sprintf(&output_msg[output_msg_len], 
        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,", 
        OUTPUT_TWR_CIR_TYPE, my_index, src_id, seq_no, distance_int, max_noise, 
        std_noise, max_growth_cir, rx_pream_count, rq, fp_power, rx_power, current_time);
    int first_path_index = (int) get_first_peak();

    // // An example: cir_start=720, cir_end=760, you should collect [720,759]
    int cir_start = first_path_index - cir_left_part;
    int cir_end = first_path_index + cir_right_part; // Make sure end is not included

    // Serial.print(cir_start);
    // Serial.print(",");
    // Serial.print(cir_end);
    int n_cir_taps = cir_end - cir_start;
    int raw_acc_mem_byte_len = 4 * (cir_end - cir_start);

    // /* Print decimal cir*/
    int16_t RealData = 0;
    int16_t ImaginaryData = 0;
    getAccMemYifeng(raw_acc_mem, cir_start, cir_end);//myAcc will contain 16 bit real imaginary pairs
    for (int i = 0 ; i< n_cir_taps; i++){
        RealData = (((uint16_t)(raw_acc_mem[(i * 4) + 1])) << 8) | raw_acc_mem[(i * 4) + 0];
        ImaginaryData = (((uint16_t)(raw_acc_mem[(i * 4) + 3])) << 8) | raw_acc_mem[(i * 4) + 2];
        output_msg_len += sprintf(&output_msg[output_msg_len], "%d,%d,", RealData, ImaginaryData);
    }

    Serial.println(output_msg);
    if (msg_type == PAT_REQUEST_TYPE) {
        start_pattern_recognition = 1;
    }
    return 0;
}

int send_pat_reply(uint8_t dst, uint16_t seq_no, uint16_t ack_no){
    beacon_msg[MSG_TYPE_IDX] = PAT_REPLY_TYPE;
	beacon_msg[SRC_IDX] = my_index;
	beacon_msg[DST_IDX] = dst;
	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
	prev_send_seq[dst] = seq_no;

    DW1000.newTransmit();
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
    txTime = DW1000.setDelay(deltaTime);
	generic_send(beacon_msg, sizeof(beacon_msg));

    while(!sendComplete);
    sendComplete = false;
    return NO_ERROR;
}

int receive_pat_shortbeacon(){
    char output_msg[1024];
    int output_msg_len = 0;
	uint8_t msg_type = rx_buffer[MSG_TYPE_IDX];
	uint8_t src_id = rx_buffer[SRC_IDX];
	uint8_t dst_id = rx_buffer[DST_IDX];
	uint16_t seq_no = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
	uint16_t ack_no = rx_buffer[ACK_IDX] + ((rx_buffer[ACK_IDX+1]) << 8);
    if (msg_type != TWR_BEACON_TYPE && msg_type != PAT_SHORTBEACON_TYPE){
        Serial.print("[receive_pat_shortbeacon()] Wrong type: ");
        Serial.println(msg_type, HEX);
        return ERROR_GENERAL;
    }
	if(dst_id != my_index){
        Serial.print("[receive_pat_shortbeacon()] Wrong dst: ");
        Serial.println(dst_id);
		return ERROR_GENERAL;
	}
    if (n_cir_verifier < MAX_CIR_VERIFIER) {
        uint16_t max_noise = get_max_noise();
        uint16_t std_noise = get_std_noise();
        uint16_t max_growth_cir = get_max_growth_cir();
        uint16_t rx_pream_count = get_rx_pream_count();
        double rq = DW1000.getReceiveQuality();
        double fp_power = DW1000.getFirstPathPower();
        double rx_power = DW1000.getReceivePower();
        double current_time = get_time_ms();
        output_msg_len += sprintf(&output_msg[output_msg_len], 
            "%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,", 
            OUTPUT_CIR_VERIFIER_TYPE, my_index, src_id, seq_no, max_noise, std_noise, max_growth_cir,
            rx_pream_count, rq, fp_power, rx_power, current_time);
        int first_path_index = (int) get_first_peak();

        // An example: cir_start=720, cir_end=760, you should collect [720,759]
        int cir_start = first_path_index - cir_left_part;
        int cir_end = first_path_index + cir_right_part; // Make sure end is not included

        int n_cir_taps = cir_end - cir_start;
        int raw_acc_mem_byte_len = 4 * (cir_end - cir_start);

        /* Print decimal cir*/
        int16_t RealData = 0;
        int16_t ImaginaryData = 0;
        getAccMemYifeng(raw_acc_mem, cir_start, cir_end);//myAcc will contain 16 bit real imaginary pairs
        for (int i = 0 ; i< n_cir_taps; i++){
            RealData = (((uint16_t)(raw_acc_mem[(i * 4) + 1])) << 8) | raw_acc_mem[(i * 4) + 0];
            ImaginaryData = (((uint16_t)(raw_acc_mem[(i * 4) + 3])) << 8) | raw_acc_mem[(i * 4) + 2];
            output_msg_len += sprintf(&output_msg[output_msg_len], "%d,%d,", RealData, ImaginaryData);
        }
        Serial.println(output_msg);
    }
    return NO_ERROR;
}

int send_pat_info_start(uint8_t dst, uint16_t seq_no, uint16_t ack_no){
    memset(beacon_msg, 0, sizeof(beacon_msg));
	beacon_msg[MSG_TYPE_IDX] = PAT_INFO_START_TYPE;
	beacon_msg[SRC_IDX] = my_index;
	beacon_msg[DST_IDX] = dst;
	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
	prev_send_seq[dst] = seq_no;

    DW1000.newTransmit();
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
	generic_send(beacon_msg, sizeof(beacon_msg));
    while(!sendComplete);
    sendComplete = false;
    return NO_ERROR;
}

int receive_pat_info_ack(){
    uint8_t msg_type = rx_buffer[MSG_TYPE_IDX];
	uint8_t src_id = rx_buffer[SRC_IDX];
	uint8_t dst_id = rx_buffer[DST_IDX];
	uint16_t seq_no = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
	uint16_t ack_no = rx_buffer[ACK_IDX] + ((rx_buffer[ACK_IDX+1]) << 8);
    if (msg_type != PAT_INFO_ACK_TYPE){
        Serial.print("[receive_pat_info_ack()] Wrong type: ");
        Serial.println(msg_type, HEX);
        return ERROR_GENERAL;
    }
	if(dst_id != my_index){
        Serial.print("[receive_pat_info_ack()] Wrong dst: ");
        Serial.println(dst_id);
		return ERROR_GENERAL;
	}
    return NO_ERROR;
}

int send_pat_info(uint8_t dst, uint16_t seq_no, uint16_t ack_no, int imu_from_idx){
    memset(beacon_msg, 0, sizeof(beacon_msg));
	beacon_msg[MSG_TYPE_IDX] = PAT_INFO_TYPE;
	beacon_msg[SRC_IDX] = my_index;
	beacon_msg[DST_IDX] = dst;
	beacon_msg[SEQ_IDX] = ((seq_no) & 0xFF);
	beacon_msg[SEQ_IDX + 1] = ((seq_no >> 8) & 0xFF);
	beacon_msg[ACK_IDX] = ((ack_no) & 0xFF);
	beacon_msg[ACK_IDX + 1] = ((ack_no >> 8) & 0xFF);
    prev_send_seq[dst] = seq_no;

    int all_imu_samples = MAX_IMU_SAMPLES;
    beacon_msg[PAT_N_ALL_IMU_COUNT_IDX] = ((all_imu_samples) & 0xFF);
	beacon_msg[PAT_N_ALL_IMU_COUNT_IDX + 1] = ((all_imu_samples >> 8) & 0xFF);
    beacon_msg[PAT_FROM_IMU_IDX] = ((imu_from_idx) & 0xFF);
	beacon_msg[PAT_FROM_IMU_IDX + 1] = ((imu_from_idx >> 8) & 0xFF);

    int imu_to_idx = imu_from_idx + MAX_IMU_EACH_PACKET_TYPICAL;
    imu_to_idx = (imu_to_idx > MAX_IMU_SAMPLES) ? MAX_IMU_SAMPLES : imu_to_idx;
    beacon_msg[PAT_TO_IMU_IDX] = ((imu_to_idx) & 0xFF);
	beacon_msg[PAT_TO_IMU_IDX + 1] = ((imu_to_idx >> 8) & 0xFF);

    uint8_t is_complete =  (imu_to_idx == MAX_IMU_SAMPLES) ? 1 : 0;
    beacon_msg[PAT_INFO_COMPLETE_BIT_IDX] = is_complete;

    embed_imus(&beacon_msg[PAT_N_IMU_IDX], imu_buffer, imu_from_idx, 
        imu_to_idx);

    DW1000.newTransmit();
    DW1000Time txTime;
    DW1000Time deltaTime = DW1000Time(FIXED_DELAY, DW1000Time::MICROSECONDS);
    txTime = DW1000.setDelay(deltaTime);
	generic_send(beacon_msg, sizeof(beacon_msg));

    while(!sendComplete);
    sendComplete = false;
    return NO_ERROR;
}

int receive_pat_info_imu_ack(){
    uint8_t msg_type = rx_buffer[MSG_TYPE_IDX];
	uint8_t src_id = rx_buffer[SRC_IDX];
	uint8_t dst_id = rx_buffer[DST_IDX];
	uint16_t seq_no = rx_buffer[SEQ_IDX] + ((rx_buffer[SEQ_IDX+1]) << 8);
	uint16_t ack_no = rx_buffer[ACK_IDX] + ((rx_buffer[ACK_IDX+1]) << 8);

    if (msg_type != PAT_INFO_IMU_ACK_TYPE){
        Serial.print("[receive_pat_info_imu_ack()] Wrong type: ");
        Serial.println(msg_type, HEX);
        return ERROR_GENERAL;
    }
	if(dst_id != my_index){
        Serial.print("[receive_pat_info_imu_ack()] Wrong dst: ");
        Serial.println(dst_id);
		return ERROR_GENERAL;
	}

    Serial.print("PC sends seq: ");
    Serial.print(seq_no);
    Serial.print("Previously, PC received my seq: ");
    Serial.println(ack_no);
    
    if (ack_no == prev_send_seq[src_id]) {
        return RET_RECEIVE_IMU_ACK;
    } else {
        Serial.println("Wrong ack");
        return RET_RECEIVE_WRONG_ACK;
    }
    return NO_ERROR;
}

#endif
