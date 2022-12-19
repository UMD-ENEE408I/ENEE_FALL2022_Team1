#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Encoder.h>
#include "Wifi.h"
#include <WiFiUdp.h>
#include <iostream>
#include <string>
using namespace std;

// Connect to wifi
// Lab router
char ssid[] = "408ITerps"; //SSID of your network
char pass[] = "goterps2022"; //password of your WPA Network

WiFiUDP udp; //The udp library class
unsigned int localPort = 2390; // local port to listen on
char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP udp2;
unsigned int localPort2 = 2391;
char packetBuffer2[255];

// Precomputed song data
float weights[] = {7,10,14,17,21,25,30,34,37,40,43,44,45,44,43,41,38,35,33,32,31,30,29,29,28,27,25,24,23,22,22,21,21,20,19,19,20,22,25,29,34,39,43,47,49,50,49,47,44,41,37,34,31,29,28,27,27,26,26,25,25,24,24,25,26,28,30,33,35,38,40,43,47,51,55,60,64,68,70,71,70,67,63,58,53,48,42,38,34,31,29,28,27,27,27,27,27,28,29,30,32,34,37,40,42,44,46,47,49,51,54,56,59,61,63,64,64,63,61,58,55,51,47,43,40,38,37,36,36,36,35,34,33,32,31,31,32,33,35,37,38,39,39,40,42,44,47,51,54,58,60,61,61,59,56,53,49,44,40,37,34,33,33,33,34,35,36,36,35,34,33,32,32,32,32,33,34,34,35,35,37,40,43,48,53,58,63,67,69,70,68,65,61,56,51,47,43,41,39,39,39,40,40,40,40,40,41,42,43,45,46,47,48,48,47,47,47,48,50,52,55,58,60,62,63,63,62,60,57,54,50,47,44,41,39,38,37,35,34,33,32,32,32,33,35,37,39,42,43,44,44,45,46,48,50,52,54,56,57,58,58,57,55,53,51,48,45,43,40,39,38,38,38,39,40,41,42,43,44,46,48,50,51,52,53,53,52,51,51,51,52,53,55,56,57,58,58,57,55,53,50,46,42,39,36,34,33,33,34,35,36,36,35,35,34,33,33,34,35,36,36,37,37,37,38,40,43,46,51,55,59,62,64,64,63,61,57,53,50,46,44,42,42,42,43,44,44,44,44,44,43,44,44,45,46,46,45,45,44,43,43,45,47,50,54,58,61,63,64,63,61,58,55,51,47,43,39,37,35,34,34,33,33,33,33,33,33,34,35,36,37,38,39,40,40,41,43,45,48,51,54,57,60,62,63,63,62,61,59,57,54,52,49,48,47,46,46,46,46,45,44,43,43,42,41,41,41,40,40,39,38,37,38,39,41,44,47,51,53,55,56,56,56,55,53,51,49,47,45,43,42,42,42,42,42,42,42,41,41,40,40,40,41,42,43,43,43,44,44,45,47,48,50,51,52,52,52,51,50,49,48,47,46,46,46,46,47,48,50,50,51,51,50,49,48,46,44,43,42,41,41,41,41,42,43,46,49,53,58,63,67,70,72,73,74,73,72,70,68,66,64,62,60,59,58,57,56,55,54,53,52,52,51,51,52,52,52,52,52,53,54,55,58,61,64,68,71,74,75,76,75,74,73,72,70,68,67,66,65,65,64,64,63,62,61,59,58,57,56,56,55,55,54,54,53,54,54,56,57,60,62,64,66,67,67,67,66,64,63,61,59,59,58,59,59,60,61,62,62,62,61,60,58,57,55,54,53,52,51,50,50,49,49,50,52,54,56,59,62,64,66,67,68,67,67,66,66,65,65,64,64,64,64,63,61,60,59,58,57,57,57,58,59,60,61,62,63,64,66,67,69,70,71,72,72,72,71,69,67,65,62,60,57,56,54,54,54,54,54,54,54,54,54,54,54,55,56,57,59,60,61,62,61,61,61,61,62,62,63,64,65,66,66,66,65,64,63,62,61,61,60,60,60,60,60,60,59,58,57,56,55,54,54,53,53,53,53,53,53,54,55,57,59,62,64,67,69,70,71,71,70,69,68,66,64,63,62,62,63,63,63,63,63,62,61,59,57,56,55,54,53,53,53,53,53,53,54,55,57,59,61,63,65,67,68,68,67,66,64,62,60,58,56,56,55,56,56,56,57,57,58,59,61,62,64,66,67,68,68,68,67,65,63,62,60,58,55,52,48,44,39,35,30,26,23,21,19,18,17,16,16,17,17,18,19,20,22,23,26,28,31,35,38,41,44,46,48,50,50,50,50,48,46,43,40,36,33,30,28,26,24,23,21,20,19,18,17,17,16,16,16,17,18,20,23,27,30,34,37,40,43,46,49,51,53,56,57,59,59,59,59,58,58,57,56,55,53,52,49,47,44,42,40,39,39,39,39,38,37,36,35,35,35,35,36,36,37,37,37,38,39,41,43,47,51,55,59,62,63,63,62,59,56,53,49,46,44,41,40,39,38,37,36,35,35,35,35,36,38,40,42,43,45,46,47,48,49,51,53,56,58,60,62,63,63,61,59,56,52,48,44,41,37,35,33,32,31,31,31,32,33,35,37,39,42,45,48,50,51,52,52,51,51,52,53,54,56,57,58,59,59,58,56,54,52,49,47,44,42,40,38,38,37,37,37,38,39,41,43,45,48,50,52,54,55,55,55,55,55,56,56,57,58,58,58,58,57,56,55,53,51,49,46,44,41,40,39,39,39,39,39,39,38,36,35,34,34,35,36,37,39,40,41,42,44,46,49,53,58,62,66,69,71,70,69,66,62,58,54,50,46,43,40,38,36,35,33,32,31,31,31,31,32,33,35,37,39,41,42,44,46,49,52,55,59,62,65,66,66,65,63,60,57,53,50,47,44,42,41,40,39,39,39,38,38,39,39,40,41,42,44,45,46,47,47,48,49,50,52,54,56,58,60,61,62,62,61,61,60,58,57,55,53,51,50,48,47,46,45,45,45,45,45,46,47,48,49,50,51,51,51,52,52,53,55,56,57,58,59,60,60,59,58,56,55,53,50,48,46,44,42,41,40,40,39,37,36,35,34,33,33,34,35,37,39,41,42,43,44,44,44,43,42,41,39,36,34,30,27,25,23,22,22,22,24,26,28,31,32,33,34,34,34,33,33,32,31,31,30,30,30,30,31,33,35,39,43,47,52,56,60,64,68,71,74,77,78,79,78,76,74,71,68,66,64,63,62,61,61,60,60,60,60,60,60,61,61,61,61,61,61,62,63,64,66,68,69,70,71,71,70,69,68,66,65,64,63,63,63,63,63,63,62,62,62,61,61,61,61,61,61,61,61,60,60,60,60,60,61,63,64,66,68,69,69,69,69,68,67,66,65,64,63,63,63,63,63,64,64,64,64,64,64,64,63,63,63,63,63,63,62,62,62,62,62,63,64,65,67,68,68,68,67,66,64,63,61,60,60,59,59,60,60,61,61,62,63,64,65,65,66,67,68,68,69,69,69,68,68,68,68,68,68,68,68,68,68,67,65,64,62,60,59,58,58,59,60,61,62,63,63,63,63,63,62,62,62,62,63,63,63,63,63,63,63,63,64,65,66,66,67,67,66,65,64,63,63,63,63,63,64,65,65,66,66,67,66,66,65,64,63,62,61,60,59,58,58,58,58,59,61,63,65,67,69,70,71,72,71,70,69,68,66,64,63,62,62,62,63,64,65,66,66,67,66,66,65,64,63,63,63,62,62,61,61,60,60,60,61,63,64,66,68,69,69,69,68,67,66,65,64,63,63,62,62,62,62,61,61,61,62,63,63,64,65,65,66,65,65,65,64,64,65,66,67,68,70,71,73,74,74,74,73,72,71,70,69,68,67,67,66,66,66,65,64,63,62,61,60,59,59,58,58,59,59,60,61,63,65,67,69,72,74,76,77,77,77,76,75,73,72,71,70,69,69,69,69,70,70,69,69,68,67,66,65,63,62,60,59,58,58,57,57,58,59,61,63,65,67,68,69,70,70,70,69,68,66,65,64,63,62,62,62,62,62,62,62,62,62,62,62,61,61,62,62,63,64,64,65,66,67,69,70,72,73,75,76,76,76,75,74,72,69,67,65,63,62,61,61,61,61,61,61,62,63,64,64,65,67,68,69,69,69,69,68,67,65,63,61,58,55,52,49,45,40,36,32,27,23,20,18,16,16,18,20,22,25,27,28,28,28,27,26,24,23,23,23,24,25,26,27,28,29,31,34,37,39,42,45,46,46,45,43,41,38,36,34,33,33,33,33,34,34,34,33,32,31,30,29,29,29,29,30,30,31,32,32,33,35,36,38,40,42,44,46,47,48,47,46,43,41,37,34,31,28,27,26,26,27,27,28,28,29,29,29,30,31,32,34,36,37,38,38,37,37,37,37,38,39,40,41,42,41,40,37,35,32,30,27,26,25,24,25,25,26,26,26,26,26,26,26,26,26,27,28,28,28,28,28,29,31,34,37,41,45,49,52,54,55,55,54,52,50,48,45,42,39,36,34,33,32,32,32,33,34,35,35,36,37,38,39,41,42,43,43,44,44,45,46,47,49,51,52,54,54,54,53,52,51,49,48,47,45,44,42,41,39,36,34,32,31,29,29,29,30,32,35,37,40,43,45,47,49,51,53,55,56,57,57,57,55,53,51,48,46,44,42,41,40,40,40,40,41,41,41,41,41,40,40,39,38,38,38,38,38,37,36,36,36,36,38,40,43,46,48,50,51,51,50,49,47,45,44,42,40,38,35,32,29,25,22,20,19,18,18,18,18,17,17,17,17,17,17,18,19,19,20,19,19,17,16,14,12,9,7,5,5,5,5,5,5,5,7,12,17,23,29,35,40,45,49,53,56,58,59,59,59,59,59,59,59,61,62,65,67,70,72,75,77,78,79,79,78,78,76,75,73,71,70,69,68,68,67,67,66,66,66,66,67,68,69,70,70,70,70,69,69,69,70,70,71,72,72,73,73,72,72,71,70,70,70,70,69,69,69,69,68,67,67,66,65,65,64,64,65,65,66,67,67,68,68,69,69,70,71,71,72,72,72,71,70,68,67,65,63,62,61,60,60,60,61,61,62,63,63,63,63,63,63,63,64,64,65,66,67,67,68,68,69,70,71,72,73,75,76,76,76,75,74,72,70,68,66,64,62,61,61,60,60,60,60,60,61,62,63,65,66,68,69,70,70,70,70,70,70,70,71,71,72,73,73,73,73,72,71,69,67,65,63,62,61,59,59,58,58,58,59,59,60,60,62,63,64,66,67,69,69,70,70,70,69,69,69,69,69,69,69,68,68,67,67,66,66,66,65,66,66,66,67,67,67,67,67,67,67,67,67,67,67,67,67,68,68,68,69,69,70,70,71,72,72,73,73,72,72,71,70,68,67,66,64,64,64,64,65,66,67,67,68,67,67,66,66,65,64,64,64,64,64,63,63,63,63,64,65,66,68,70,72,73,74,74,73,71,70,68,67,66,66,65,66,66,66,67,67,67,68,68,68,69,69,69,68,68,68,67,66,66,66,67,67,68,69,71,72,73,73,73,73,73,73,73,72,72,72,72,71,71,70,69,68,67,65,64,63,61,61,60,60,61,62,63,64,65,66,67,68,69,71,72,73,73,73,72,71,69,68,66,65,64,64,64,64,64,64,64,64,65,65,66,67,67,68,68,69,69,69,69,70,70,71,72,73,74,75,76,77,77,77,76,75,73,71,70,68,66,65,65,64,64,65,65,64,64,64,63,63,62,62,61,61,60,60,61,61,62,63,65,66,68,70,71,72,73,73,72,71,69,67,65,64,63,62,62,63,63,64,65,65,66,65,65,65,64,64,63,63,63,62,61,61,61,61,61,63,64,66,69,71,73,74,74,73,73,71,70,68,67,66,65,64,63,62,61,61,60,60,60,60,61,61,62,63,63,64,64,65,65,66,66,67,68,69,70,70,70,70,69,68,66,65,64,64,64,64,65,66,67,68,69,70,70,71,71,72,71,71,71,70,69,69,68,68,68,68,69,69,70,71,72,73,74,74,74,74,73,72,70,68,67,66,65,64,64,63,63,62,61,60,59,58,57,56,56,56,57,57,58,59,61,62,64,67,69,72,74,76,78,79,79,78,76,74,71,69,67,65,64,63,63,64,64,65,65,65,66,66,66,66,66,65,64,64,62,61,59,57,55,52,49,46,42,38,34,29,24,18,13,9,5,2,0,0,0,1,2,4,5,6,7,6,6,5,4,3,1,0,0,0,0,0,0,0,0,0,0,0,1,1,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3};
int len_weights = 2614;
float song_dur = 217.97;
float beat_pd = 0.74;

// IMU (rotation rate and acceleration)
Adafruit_MPU6050 mpu;

// Buzzer pin which we will use for indicating IMU initialization failure
const unsigned int BUZZ = 26;
const unsigned int BUZZ_CHANNEL = 0;

// Need these pins to turn off light bar ADC chips
const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;

// Battery voltage measurement constants
const unsigned int VCC_SENSE = 27;
const float ADC_COUNTS_TO_VOLTS = (2.4 + 1.0) / 1.0 * 3.3 / 4095.0;

// Motor encoder pins
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;

// Motor power pins
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;

// Motor PWM channels
const unsigned int M1_IN_1_CHANNEL = 8;
const unsigned int M1_IN_2_CHANNEL = 9;
const unsigned int M2_IN_1_CHANNEL = 10;
const unsigned int M2_IN_2_CHANNEL = 11;

const int M_PWM_FREQ = 5000;
const int M_PWM_BITS = 8;
const unsigned int MAX_PWM_VALUE = 255; // Max PWM given 8 bit resolution

float METERS_PER_TICK = (3.14159 * 0.031) / 360.0;
float TURNING_RADIUS_METERS = 4.3 / 100.0; // Wheels are about 4.3 cm from pivot point

void configure_imu() {
	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
			ledcWriteNote(BUZZ_CHANNEL, NOTE_C, 4);
			delay(500);
			ledcWriteNote(BUZZ_CHANNEL, NOTE_G, 4);
			delay(500);
		}
  	}
	Serial.println("MPU6050 Found!");
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
	mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
	mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
}

void read_imu(float& w_z) {
	sensors_event_t a, g, temp;
	mpu.getEvent(&a, &g, &temp);
	w_z = g.gyro.z;
}

void est_imu_bias(float& E_w_z, int N_samples) {
	float E_w_z_acc = 0.0;
	for (unsigned int i = 0; i < N_samples; i++) {
		float w_z;
		read_imu(w_z);
		E_w_z_acc += w_z;
		delay(5);
	}
	E_w_z = E_w_z_acc / N_samples;
}

void configure_motor_pins() {
	ledcSetup(M1_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
	ledcSetup(M1_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
	ledcSetup(M2_IN_1_CHANNEL, M_PWM_FREQ, M_PWM_BITS);
	ledcSetup(M2_IN_2_CHANNEL, M_PWM_FREQ, M_PWM_BITS);

	ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
	ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
	ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
	ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);
}

// Positive means forward, negative means backwards
void set_motors_pwm(float left_pwm, float right_pwm) {
	if (isnan(left_pwm)) left_pwm = 0.0;
	if (left_pwm  >  255.0) left_pwm  =  255.0;
	if (left_pwm  < -255.0) left_pwm  = -255.0;
	if (isnan(right_pwm)) right_pwm = 0.0;
	if (right_pwm >  255.0) right_pwm =  255.0;
	if (right_pwm < -255.0) right_pwm = -255.0;

	if (left_pwm > 0) {
		ledcWrite(M1_IN_1_CHANNEL, 0);
		ledcWrite(M1_IN_2_CHANNEL, (uint32_t)(left_pwm));
	} else {
		ledcWrite(M1_IN_1_CHANNEL, (uint32_t)-left_pwm);
		ledcWrite(M1_IN_2_CHANNEL, 0);
	}

	if (right_pwm > 0) {
		ledcWrite(M2_IN_1_CHANNEL, 0);
		ledcWrite(M2_IN_2_CHANNEL, (uint32_t)(right_pwm));
	} else {
		ledcWrite(M2_IN_1_CHANNEL, (uint32_t)-right_pwm);
		ledcWrite(M2_IN_2_CHANNEL, 0);
	}
}

float update_pid(float dt, float kp, float ki, float kd,
                 float x_d, float x,
                 float& int_e, float abs_int_e_max, // last_x and int_e are updated by this function
                 float& last_x) {
	// Calculate or update intermediates
	float e = x_d - x; // Error

	// Integrate error with anti-windup
	int_e = int_e + e * dt;
	if (int_e >  abs_int_e_max) int_e =  abs_int_e_max;
	if (int_e < -abs_int_e_max) int_e = -abs_int_e_max;

	// Take the "Derivative of the process variable" to avoid derivative spikes if setpoint makes step change
	// with abuse of notation, call this de
	float de = -(x - last_x) / dt;
	last_x = x;

	float u = kp * e + ki * int_e + kd * de;
	return u;
}

// a smooth and interesting trajectory
////CHANGE X AND Y TO CHANGE TRAJECTORY
// https://en.wikipedia.org/wiki/Lemniscate_of_Bernoulli
void lemniscate_of_bernoulli(float t, float r, float& x, float& y, int move, float period){
	// All moves have period, radius that were passed in, and start at (r, 0).
	
	// Circle
	if (move == 1) {
		//Serial.printf("%d+circle++",move);
		x = r * cos((2*3.14/period)*t);
		y = r * sin((2*3.14/period)*t);
	}
	// Lemniscate of Bernoulli
	else if (move == 2) {
		float sin_t = sin(t*((2*3.14)/period));
		float den = 1 + sin_t * sin_t;
		x = r * cos(t*(2*3.14)/period) / den;
		y = r * sin(t*(2*3.14)/period) * cos(t*(2*3.14)/period) / den;
	}
	// Triquetra knot
	else if (move == 3) {
		x = r * cos((3*3.14/period)*t) * cos((3.14/period)*t);
		y = r * cos((3*3.14/period)*t) * sin((3.14/period)*t);
	}
	// Flower with 7 petals
	else if (move == 4) {
		x = r * 0.8 * (cos((2*3.14/period)*t) + 0.25 * cos((16*3.14/period)*t));
		y = r * 0.8 * (sin((2*3.14/period)*t) + 0.25 * sin((16*3.14/period)*t));
	}
	// Stay at origin
	else {
		x = r;
		y = 0;
	}
}

// Signed angle from (x0, y0) to (x1, y1)
// assumes norms of these quantities are precomputed
float signed_angle(float x0, float y0, float n0, float x1, float y1, float n1) {
	float normed_dot = (x1 * x0 + y1 * y0) / (n1 * n0);
	if (normed_dot > 1.0) normed_dot = 1.0; // Possible because of numerical error
	float angle = acosf(normed_dot);

	// use cross product to find direction of rotation
	// https://en.wikipedia.org/wiki/Cross_product#Coordinate_notation
	float s3 = x0 * y1 - x1 * y0;
	if (s3 < 0) angle = -angle;

	return angle;
}

void setup() {
	// Stop the right motor by setting pin 14 low
	// this pin floats high or is pulled
	// high during the bootloader phase for some reason
	pinMode(14, OUTPUT);
	digitalWrite(14, LOW);
	delay(100);

	Serial.begin(115200);
	delay(10);

	// We start by connecting to a WiFi network
	Serial.println();
	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(ssid);
	WiFi.begin(ssid, pass);
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}
	Serial.println("");
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());

    udp.begin(localPort); // For receiving
	udp2.begin(localPort2);

	// Disalbe the lightbar ADC chips so they don't hold the SPI bus used by the IMU
	pinMode(ADC_1_CS, OUTPUT);
	pinMode(ADC_2_CS, OUTPUT);
	digitalWrite(ADC_1_CS, HIGH);
	digitalWrite(ADC_2_CS, HIGH);

	ledcAttachPin(BUZZ, BUZZ_CHANNEL);
	pinMode(VCC_SENSE, INPUT);

	configure_motor_pins();
	configure_imu();

	Serial.println("Starting!");
}

void loop() {
	// Create the encoder objects after the motor has
	// stopped, else some sort exception is triggered
	Encoder enc1(M1_ENC_A, M1_ENC_B);
	Encoder enc2(M2_ENC_A, M2_ENC_B);

	// Loop period
	int target_period_ms = 2; // Loop takes about 3 ms so a delay of 2 gives 200 Hz or 5ms

	//dummy inputs for the lemniscate of bernoulli
	int move = 1;
	float period = 7;
	// States used to calculate target velocity and heading
	float lemniscate_a = 0.35; // Radius
	//float lemniscate_t_scale = 2.0; // speedup factor
	float x0, y0;
	lemniscate_of_bernoulli(0.0, lemniscate_a, x0, y0,move,period);
	x0 += 0.15;
	float last_x, last_y;
	lemniscate_of_bernoulli(-target_period_ms / 1000.0, lemniscate_a, last_x, last_y,move,period);
	last_x += 0.15;
	float last_dx = (x0 - last_x) / ((float)target_period_ms / 1000.0);
	float last_dy = (y0 - last_y) / ((float)target_period_ms / 1000.0);
	float last_target_v = sqrtf(last_dx * last_dx + last_dy * last_dy);
	float target_theta = 0.0; // This is an integrated quantity

	// Motors are controlled by a position PID
	// with inputs interpreted in meters and outputs interpreted in volts
	// integral term has "anti-windup"
	// derivative term uses to derivative of process variable (wheel position)
	// instead of derivative of error in order to avoid "derivative kick"
	float kp_left = 200.0;
	float ki_left = 20.0;
	float kd_left = 20.0;
	float kf_left = 10.0;
	float target_pos_left  = 0.0;
	float last_pos_left = 0.0;
	float integral_error_pos_left = 0.0;
	float max_integral_error_pos_left = 1.0 * 8.0 / ki_left; // Max effect is the nominal battery voltage

	float kp_right = 200.0;
	float ki_right = 20.0;
	float kd_right = 20.0;
	float kf_right = 10.0;
	float last_pos_right = 0.0;
	float target_pos_right = 0.0;
	float integral_error_pos_right = 0.0;
	float max_integral_error_pos_right = 1.0 * 8.0 / ki_right; // Max effect is the nominal battery voltage

	// IMU Orientation variables
	float theta = 0.0;
	float bias_omega;
	// Gain applied to heading error when offseting target motor velocities
	// currently set to 360 deg/s compensation for 90 degrees of error
	float ktheta = (2 * 3.14159) / (90.0 * 3.14159 / 180.0);
	est_imu_bias(bias_omega, 500);// Could be expanded for more quantities

	//Get incoming packet
	int curr_mv = 1; // Dance move number being executed
	int new_mv = curr_mv; //  New received move

	// For move 30
	int special_mv = 1;
	int new_special_mv = special_mv;
	float special_param = 10;
	float new_special_param = special_param;
	bool transition_special = false;
	bool just_transitioned = true;
	int flip = 1; // 0 for backward, 1 for forward
	bool first_move = true; // If first special dance move

	float curr_param = 8; // Param for move
	float new_param = curr_param;
	float t_offset = 0; // Offset seen by function to generate x, y
	char incomingPacket[256];

	// The real "loop()"
	// time starts from 0
	float start_t = (float)micros() / 1000000.0;
	float last_t = -target_period_ms / 1000.0; // Offset by expected looptime to avoid divide by zero

	float special_t = -1; // Special time for changing speed through 
	float offset_in_song = 0; // Also for speed changing loop
	float corresp_t = 0; // t at which that offset happened

	// Cumulative x and y errors
	float acc_err_x = 0;
	float acc_err_y = 0;
	// Model drift of origin
	float drift_x = 0;
	float drift_y = 0;
	// Ideal position without drift
	float last_x_ideal = last_x;
	float last_y_ideal = last_y;

	bool endOfDance = false; // Stop dancing
  
	while (true) {
		//get packet
		int packetSize = udp.parsePacket();
     	// Once packet received
		if (packetSize) {
			Serial.print("Received packet of size ");
			Serial.println(packetSize);
			Serial.print("From ");
			IPAddress remoteIp = udp.remoteIP();
			Serial.print(remoteIp);
			Serial.print(", port ");
			Serial.println(udp.remotePort());
		
			// read the packet into packetBufffer
			int len = udp.read(packetBuffer, 255);
			if (len > 0) {
				packetBuffer[len] = 0;
			}

			// Print/scan contents
			Serial.print("Contents: ");
			Serial.println(packetBuffer);
			sscanf(packetBuffer, "%d %f %d %f %d", &new_mv, &new_param, &new_special_mv, &new_special_param, &flip); // Read data into new_mv and param
		}
		// Get other packets
		//get packet
		int packetSize2 = udp2.parsePacket();
     	// Once packet received
		float pos_x;
		float pos_y;
		float heading_x;
		float heading_y;
		if (packetSize2) {
			Serial.print("Received packet of size ");
			Serial.println(packetSize);
			Serial.print("From ");
			IPAddress remoteIp = udp2.remoteIP();
			Serial.print(remoteIp);
			Serial.print(", port ");
			Serial.println(udp2.remotePort());
		
			// read the packet into packetBufffer
			int len = udp2.read(packetBuffer2, 255);
			if (len > 0) {
				packetBuffer2[len] = 0;
			}

			sscanf(packetBuffer2, "%f %f %f %f", &pos_x, &pos_y, &heading_x, &heading_y);
			// Convert to m
			pos_x = pos_x / 100;
			pos_y = pos_y / 100;

			// Now calculate error in robot's x, y coordinates, in meters.
			// Assuming robot starts off at (4 ft, 1 ft) in arena coordinates, so
			// origin is at (4 * 30.48 cm, 30.48 + lemnsicate_a cm).
			// (Robot's origin is lemniscate_a above starting point.)
			// x, y values in robot's coordinates
			float actual_x = (lemniscate_a + (3.5 * 30.48)/100) - pos_y;
			float actual_y = pos_x - (4 * 30.48/100);
			// Error: where robot is vs. where it should be.
			float err_x = actual_x - last_x_ideal;
			float err_y = actual_y - last_y_ideal;
			Serial.printf("Received pos: %f %f %f %f\n", pos_x, pos_y, heading_x, heading_y);

			// Update accumulated errors based on this new data
			acc_err_x = 0.7 * acc_err_x + 0.3 * err_x;
			acc_err_y = 0.7 * acc_err_y + 0.3 * err_y;

			Serial.printf("errors   %f %f %f %f\n", acc_err_x, acc_err_y, err_x, err_y);
		}
		// If not tag detected, decrease accumulated errors
		else if (!packetSize2) {
			acc_err_x *= 0.9995;
			acc_err_y *= 0.9995;
		}
		// Model where robot has drifted to
		drift_x += 0.0008 * acc_err_x;
		drift_y += 0.0008 * acc_err_y;
		
		// Get the time elapsed
		float t = ((float)micros()) / 1000000.0 - start_t;
		float dt = ((float)(t - last_t)); // Calculate time since last update

		// Detect transition - end of next period, when a dance move is completed.
		// Transitions are when t - t_offset = 0 mod period.
		// If transition, update the mv, param, offset.
		if (fmod(t - t_offset, curr_param) <= fmod(last_t - t_offset, curr_param) || transition_special) {
			Serial.printf("TRANSITION: mv %d -> %d, param %f -> %f\n", curr_mv, new_mv, curr_param, new_param);
			curr_mv = new_mv;
			curr_param = new_param;
			t_offset = last_t;
			if (transition_special) {
				just_transitioned = true;
				// Only reset special_t if something changed
				if (special_mv != new_special_mv || special_param != new_special_param) {
					special_t = 0;
				}
				special_mv = new_special_mv;
				special_param = new_special_param;
			}
			transition_special = false;
		}
		
		// Serial.print("t "); Serial.print(t);
		// Serial.print(" dt "); Serial.print(dt * 1000.0);
		last_t = t;

		// Get the distances the wheels have traveled in meters
		// positive is forward
		float pos_left  =  (float)enc1.read() * METERS_PER_TICK;
		float pos_right = -(float)enc2.read() * METERS_PER_TICK; // Take negative because right counts upwards when rotating backwards
  
		// TODO Battery voltage compensation, the voltage sense on my mouse is broken for some reason
		// int counts = analogRead(VCC_SENSE);
		// float battery_voltage = counts * ADC_COUNTS_TO_VOLTS;
		// if (battery_voltage <= 0) Serial.println("BATTERY INVALID");
  
		// Read IMU and update estimate of heading
		// positive is counter clockwise
		float omega;
		read_imu(omega); // Could be expanded to read more things
		omega -= bias_omega; // Remove the constant bias measured in the beginning
		theta = theta + omega * dt;
		// Serial.print(" omega "); Serial.print(omega);
		// Serial.print(" theta "); Serial.print(theta);
		// Serial.print(" last_x "); Serial.print(last_x);
		// Serial.print(" last_y "); Serial.print(last_y);
		// Serial.print(" last_dx "); Serial.print(last_dx);
		// Serial.print(" last_dy "); Serial.print(last_dy);
		// Serial.print(" last tv "); Serial.print(last_target_v);

		// Calculate target forward velocity and target heading to track the lemniscate trajectory
		// of 0.5 meter radius
		float x, y;
		// Note - removed lemniscate_t_scale!
		lemniscate_of_bernoulli(t - t_offset, lemniscate_a, x, y, curr_mv, curr_param);
		x += 0.15;

		// Use precomputed song data to speed up/slow down.
		// Send packet as "30 offset_in_song mv period direction(1/0)".
		// Note - offset into song is only set the first time, then never again.
		if (curr_mv == 30) {
			// If first time loop runs, get how far in song it is
			if (first_move) {
				offset_in_song = curr_param;
				corresp_t = t;
			}
			curr_param = 10000; // Prevent any future transitions

			float ts = t - corresp_t + offset_in_song; // Time into song
			float special_t_init = special_t;

			// Now get data for relevant part of song
			int index = floor((ts / song_dur) * len_weights);
			float speedup = 1; // Time speedup
			if (index < len_weights) {
				speedup = (weights[index] + 2) / 18;
			}
			else {
				speedup = 0;
				endOfDance = true;
			}

			// If first time, set special time to current time minus offset
			if (first_move) {
				special_t = t - t_offset;
				first_move = false; // Only run this for first move
			}
			// Otherwise, increment special_t based on actual time change
			else {
				special_t += (2 * (flip - 0.5)) * dt * speedup;
			}

			// Set period (default speed) here and dance move to do
			lemniscate_of_bernoulli(special_t, 0.35, x, y, special_mv, special_param);
			x += 0.15; // To account for smaller radius

			Serial.printf("special_t=%f,   ts=%f,   speedup=%f,   x=%f,   y=%f\n", special_t, ts, speedup, x, y);

			// Check if transition
			float x_init;
			float y_init;
			lemniscate_of_bernoulli(special_t_init, 0.35, x_init, y_init, special_mv, special_param);
			x_init += 0.15;
			// Condition for transition - x based on radius. 2 conditions because may be going forward or backward.
			if (!just_transitioned && ((y >= 0 && y_init <= 0 && x >= 0.48)
					|| (y <= 0 && y_init >= 0 && x >= 0.48))) {
				transition_special = true;
			}
			just_transitioned = false;
		}

		// Serial.print(" x "); Serial.print(x);
		// Serial.print(" y "); Serial.print(y);

		//Serial.printf("actual xy %f %f\n", x, y);
		// Save x and y without drift modification, to compute error
		last_x_ideal = x;
		last_y_ideal = y;
		// Modify target x and y based on errors
		x -= drift_x;
		y -= drift_y;
		//Serial.printf("errors   %f %f\n", acc_err_x, acc_err_y);

		float dx = (x - last_x) / dt;
		float dy = (y - last_y) / dt;
		float target_v = sqrtf(dx * dx + dy * dy); // forward velocity

		// Serial.print(" dx "); Serial.print(dx);
		// Serial.print(" dy "); Serial.print(dy);
		// Serial.print(" tv "); Serial.print(target_v);

		// Compute the change in heading using the normalized dot product between the current and last velocity vector
		// using this method instead of atan2 allows easy smooth handling of angles outsides of -pi / pi at the cost of
		// a slow drift defined by numerical precision
		float target_omega = signed_angle(last_dx, last_dy, last_target_v, dx, dy, target_v) / dt;
		// Added because nan if no movement! -> stops robot from moving any further.
		if (isnan(target_omega)) {
			target_omega = 0;
		}

		// Back and forth dance move implemented here.
		// Assume corresponding param is twice beat period in s.
		if (curr_mv == 10) {
			target_omega = 0;
			if (fmod(t - t_offset, curr_param) <= curr_param / 2) {
				target_v = 0.2;
			}
			else {
				target_v = -0.2;
			}
		}
		else if (curr_mv == 11) {
			target_v = 0;
			if (fmod(t - t_offset, curr_param) <= curr_param / 2) {
				target_omega = 2.2;
				//target_v = 0.3;
			}
			else {
				target_omega = -2.2;
				//target_v = -0.3;
			}
		}

		// End of dance - stop moving
		if (endOfDance) {
			target_omega = 0;
			target_v = 0;
		}

		target_theta = target_theta + target_omega * dt;

		// Serial.print(" target_omega "); Serial.print(target_omega);
		// Serial.print(" t theta "); Serial.print(target_theta);

		last_x = x;
		last_y = y;
		last_dx = dx;
		last_dy = dy;
		last_target_v = target_v;
  
		// Calculate target motor speeds from target forward speed and target heading
		// Could also include target path length traveled and target angular velocity
		float error_theta_z = target_theta - theta;
		float requested_v = target_v;
		float requested_w = ktheta * error_theta_z;

		float target_v_left  = requested_v - TURNING_RADIUS_METERS * requested_w;
		float target_v_right = requested_v + TURNING_RADIUS_METERS * requested_w;
		target_pos_left  = target_pos_left  + dt * target_v_left;
		target_pos_right = target_pos_right + dt * target_v_right;

		// Serial.print(" tpl "); Serial.print(target_pos_left);
		// Serial.print(" pl "); Serial.print(pos_left);
		// Serial.print(" tpr "); Serial.print(target_pos_right);
		// Serial.print(" pr "); Serial.print(pos_right);

		// Left motor position PID
		float left_voltage = update_pid(dt, kp_left, ki_left, kd_left,
										target_pos_left, pos_left,
										integral_error_pos_left, max_integral_error_pos_left,
										last_pos_left);
		left_voltage = left_voltage + kf_left * target_v_left;
		float left_pwm = (float)MAX_PWM_VALUE * (left_voltage / 8.0); // TODO use actual battery voltage

		// Right motor position PID
		float right_voltage = update_pid(dt, kp_right, ki_right, kd_right,
										target_pos_right, pos_right,
										integral_error_pos_right, max_integral_error_pos_right,
										last_pos_right);
		left_voltage = right_voltage + kf_right * target_v_right;
		float right_pwm = (float)MAX_PWM_VALUE * (right_voltage / 8.0); // TODO use actual battery voltage

		// Serial.print(" l voltage " ); Serial.print(left_voltage);
		// Serial.print(" r voltage " ); Serial.print(right_voltage);

		set_motors_pwm(left_pwm, right_pwm);

		// Serial.println();
		delay(target_period_ms);
	}
}