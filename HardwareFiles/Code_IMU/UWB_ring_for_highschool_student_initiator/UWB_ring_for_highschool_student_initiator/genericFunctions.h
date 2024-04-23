#ifndef GENERIC_FUNCTIONS_H
#define GENERIC_FUNCTIONS_H

#include <DW1000.h>
#include "ProtocolConsts.h"
#include "Adafruit_LSM9DS1.h"
#include <LSM6DSOSensor.h>

extern int32_t imu_buffer[IMU_BUFFER_SIZE][7];
extern LSM6DSOSensor AccGyr;


void getAccMemYifeng(uint8_t acc_mem[], uint16_t start, uint16_t end) {
	
	//Setup the correct clocks
	byte reg[2];
	DW1000.readBytes(PMSC, PMSC_CTRL0_SUB, reg, 2);
	reg[0] = 0x48 | (reg[0] & 0xb3);
	reg[1] = 0x80 | reg[1];
	DW1000.writeBytes(PMSC, PMSC_CTRL0_SUB, &reg[0], 1);
	DW1000.writeBytes(PMSC, 0x01, &reg[1], 1);

	DW1000.readBytes(0x25, start*4+1, acc_mem, (end-start)*4);
	
	reg[0] = reg[0] & 0xb3;
	reg[1] = 0x7f & reg[1];
	DW1000.writeBytes(PMSC, PMSC_CTRL0_SUB, &reg[0], 1);
	DW1000.writeBytes(PMSC, 0x01, &reg[1], 1);
}

void set_customized_configs(){
	DW1000.setChannel(DW1000.CHANNEL_5);
	DW1000.setPulseFrequency(DW1000.TX_PULSE_FREQ_64MHZ);
	DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_128);
	DW1000.setDataRate(DW1000.TRX_RATE_6800KBPS);
	DW1000.useExtendedFrameLength(true);
}

void set_customized_configs2(){
	DW1000.setPulseFrequency(DW1000.TX_PULSE_FREQ_64MHZ);
	DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_128);
	DW1000.setDataRate(DW1000.TRX_RATE_6800KBPS);
	DW1000.setChannel(DW1000.CHANNEL_3);
	DW1000.useExtendedFrameLength(true);
}

// channel 5
void set_customized_configs3(){
	// DW1000.setPulseFrequency(DW1000.TX_PULSE_FREQ_64MHZ);
	// DW1000.setPreambleLength(DW1000.TX_PREAMBLE_LEN_128);
	// DW1000.setDataRate(DW1000.TRX_RATE_6800KBPS);
	// DW1000.setChannel(DW1000.CHANNEL_3);
	// DW1000.useExtendedFrameLength(true);
}

double get_first_peak(){
	byte fp_index_bytes[2] = {0};
	DW1000.readBytes(RX_TIME, 5, fp_index_bytes, 2);
	uint16_t tmp = (fp_index_bytes[1] << 8) + fp_index_bytes[0];
	double first_path = (tmp >> 6) + ((tmp & 0x3F) / 64.0);
	return first_path;
}

uint16_t get_max_noise(){
	uint16_t max_noise;
	byte maxNoiseBytes[2];
	DW1000.readBytes(LDE_IF, 0, maxNoiseBytes, 2);
	max_noise = (uint16_t)maxNoiseBytes[0] | ((uint16_t)maxNoiseBytes[1] << 8);
	return max_noise;
}

uint16_t get_std_noise(){
	uint16_t noise;
	byte noiseBytes[2];
	DW1000.readBytes(RX_FQUAL, STD_NOISE_SUB, noiseBytes, 2);
	noise = (uint16_t)noiseBytes[0] | ((uint16_t)noiseBytes[1] << 8);
	return noise;
}

uint16_t get_max_growth_cir(){
	uint16_t max_growth_cir;
	byte maxGrowthCirBytes[2];
	DW1000.readBytes(RX_FQUAL, CIR_PWR_SUB, maxGrowthCirBytes, 2);
	max_growth_cir = (uint16_t)maxGrowthCirBytes[0] | ((uint16_t)maxGrowthCirBytes[1] << 8);
	return max_growth_cir;
}

uint16_t get_rx_pream_count(){
	uint16_t rx_pream_count;
	byte rxFrameInfo[4];
	DW1000.readBytes(RX_FINFO, NO_SUB, rxFrameInfo, LEN_RX_FINFO);
	rx_pream_count  = (((uint16_t)rxFrameInfo[2] >> 4) & 0xFF) | ((uint16_t)rxFrameInfo[3] << 4);
	return rx_pream_count;
}

void any_msg_set_ts(uint8_t *ts_field, uint64_t ts) {
	int i;
	for (i = 0; i < TS_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

void any_msg_get_ts(const uint8_t *ts_field, uint64_t *ts) {
	int i;
	*ts = 0;
	for (i = 0; i < TS_LEN; i++) {
		*ts += ((uint64_t)ts_field[i]) << (i * 8);
	}
}

void short_msg_set(uint8_t *ts_field, uint16_t ts) {
	int i;
	for (i = 0; i < SHORT_MSG_LEN; i++) {
		ts_field[i] = (uint8_t) ts;
		ts >>= 8;
	}
}

void short_msg_get(const uint8_t *ts_field, uint16_t *ts) {
	int i;
	*ts = 0;
	for (i = 0; i < SHORT_MSG_LEN; i++) {
		*ts += ((uint64_t)ts_field[i]) << (i * 8);
	}
}

double get_time_us(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    uint64_t tmpTime_u64 = currentDWTime.getTimestamp();
    return tmpTime_u64 * TIME_UNIT * 1e6;
}

double get_time_ms(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    uint64_t tmpTime_u64 = currentDWTime.getTimestamp();
    return tmpTime_u64 * TIME_UNIT * 1e3;
}

uint16_t get_time_ms_uint16(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    uint64_t tmpTime_u64 = currentDWTime.getTimestamp();
    return (uint16_t)(tmpTime_u64 * TIME_UNIT * 1e3);
}

uint64_t get_time_u64(){
    DW1000Time currentDWTime;
    DW1000.getSystemTimestamp(currentDWTime);
    return currentDWTime.getTimestamp();
}

double get_elapsed_time_us(double startTime){
	double currentTime = get_time_us();
    return (startTime < currentTime) ? currentTime - startTime : currentTime + 17207401.0256 - startTime;
}

double get_elapsed_time_ms(double startTime){
	double currentTime = get_time_ms();
    return (startTime < currentTime) ? currentTime - startTime : currentTime + 17207.4010256 - startTime;
}

void print_uint64(uint64_t a){
	int r;
	int out[20]; int j = 0;
	while(a > 0){
		r = a % 10;
		a = (a - r) / 10;
		out[j++] = r;
	}
	for(int i = j - 1; i>= 0; i--){
		Serial.print(out[i]);
	}
}

void generic_send(uint8_t *buffer_to_send, int buffer_size) {
    
    
    uint32_t txDelay;
    
    DW1000.setData(buffer_to_send, buffer_size);
    
    DW1000.startTransmit();
}

uint64_t get_timestamp_from_packet_u64(const uint8_t *ts_field) {
	int i;
	uint64_t ts = 0;
	for (i = 0; i < TS_LEN; i++) {
		ts += ((uint64_t)(ts_field[i])) << (i * 8);
	}
	return ts;
}

void set_timestamp_to_packet_u64(uint8_t *ts_field, uint64_t ts) {
	int i;
	for (i = 0; i < TS_LEN; i++) {
		ts_field[i] = (ts & 0xFF);
		ts >>= 8;
	}
}

uint16_t increment_seq(uint16_t seq_no){
	if (seq_no == MAX_SEQ_NUMBER){
		return 1;
	} else{
		return seq_no + 1;
	}
}

bool is_next_seq(uint16_t my_seq, uint16_t current_seq){
	if (current_seq == MAX_SEQ_NUMBER){
		return (my_seq == 1);
	} else {
		return (my_seq == current_seq + 1);
	}
}


uint64_t get_rx_timestamp_u64(){
	DW1000Time rxTS;
	DW1000.getReceiveTimestamp(rxTS);
	uint64_t thisRecvTime = rxTS.getTimestamp();
	return thisRecvTime;
}

int32_t scale_imu_measurement(double a) {
	return (int32_t)((max(min(a, 1000), -1000))*10000);
}

void add_imu_to_buffer(int32_t imu_buffer[][7], int &idx, int max_samples_in_buf){
	int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.Get_X_Axes(accelerometer);
    AccGyr.Get_G_Axes(gyroscope);

	if (idx >= max_samples_in_buf) {
		idx = 0;
	}

	// The input of scale_imu_measurement must be in m/s^2 and dps
	if (idx < max_samples_in_buf){
		imu_buffer[idx][0] = scale_imu_measurement(((float)(accelerometer[0]))/1000*10);
		imu_buffer[idx][1] = scale_imu_measurement(((float)(accelerometer[1]))/1000*10);
		imu_buffer[idx][2] = scale_imu_measurement(((float)(accelerometer[2]))/1000*10);
		imu_buffer[idx][3] = scale_imu_measurement(((float)(gyroscope[0]))/1000);
		imu_buffer[idx][4] = scale_imu_measurement(((float)(gyroscope[1]))/1000);
		imu_buffer[idx][5] = scale_imu_measurement(((float)(gyroscope[2]))/1000);
		imu_buffer[idx][6] = (int32_t)(get_time_us()); // 0 ~ 18000
		Serial.print("imu meansurements:");
		Serial.print(idx);
		Serial.print(",");
		Serial.print(imu_buffer[idx][0]);
		Serial.print(",");
		Serial.print(imu_buffer[idx][1]);
		Serial.print(",");
		Serial.print(imu_buffer[idx][2]);
		Serial.print(",");
		Serial.print(imu_buffer[idx][3]);
		Serial.print(",");
		Serial.print(imu_buffer[idx][4]);
		Serial.print(",");
		Serial.println(imu_buffer[idx][5]);
		idx += 1;
	}
}


// buf should start from n_imu_idx
void embed_imus(uint8_t buf[], int32_t imu_measurements[][7], int n_imus){
	int n = (n_imus > MAX_IMU_EACH_PACKET_TWR_LOGIC) ? MAX_IMU_EACH_PACKET_TWR_LOGIC:n_imus;
	buf[0] = (uint8_t) (n & 0xFF);
	// Serial.print("n_imu is: ");
	// Serial.println(n_imus);

	int n_bytes = n_imus  * (3*ACC_MSG_LEN + 3*GYRO_MSG_LEN + IMU_TIME_MSG_LEN);
	// Serial.print("n_bytes is: ");
	// Serial.println(n_bytes);
	memcpy(&buf[1], imu_measurements, n_bytes);
}

void embed_imus(uint8_t buf[], int32_t imu_measurements[][7], int from_idx, 
	int to_idx){
	int n_imus = to_idx - from_idx;
	buf[0] = (uint8_t) (n_imus & 0xFF);
	// Serial.print("n_imu is: ");
	// Serial.println(n_imus);

	int n_bytes = n_imus  * (3*ACC_MSG_LEN + 3*GYRO_MSG_LEN + IMU_TIME_MSG_LEN);
	// Serial.print("n_bytes is: ");
	// Serial.println(n_bytes);
	// int offset = from_idx * 6 * ACC_MSG_LEN;
	memcpy(&buf[1], &imu_measurements[from_idx], n_bytes);
}
#endif