#ifndef CONFIG_H_
#define CONFIG_H_

#define pinPCC       0
#define pinSDA       4
#define pinSCL       5
#define pinDRDYInt   15
#define pinAudio 	 14
#define LED       2
#define ledsPin        14
#define NUMPIXELS 8
#define button 16


////////////////////////////////////////////////////////////////////
// Alarm configuration
// Structure to store the LED configuration
struct LedConfig {
    uint32_t color;          // Color in 0xRRGGBB format
    bool shouldBlink;        // Should the LEDs blink?
    unsigned long blinkInterval; // Blink interval in milliseconds
    float minAltitude;       // Minimum altitude for this range
    float maxAltitude;       // Maximum altitude for this range
    uint8_t brightness;      // Brightness level (0-255)
};

// LED configurations for different altitude ranges
const LedConfig configs[] = {
	{0x00FF0000, true, 100, -150, -50, 255}, 
    {0x00FF0000, true, 300, -50, 50, 50},  // Red, blinks every 500 ms, from 0 to 300 cm
    {0x00FFFF00, false, 0, 50, 100, 50}, // Yellow, no blinking, from 300 to 500 cm
    {0x0000FF00, false, 0, 100, 150, 50} // Green, no blinking, above 500 cm
};




#define APP_MODE_VARIO   11

////////////////////////////////////////////////////////////////////
// WEB CONFIGURATION PARAMETER DEFAULTS AND LIMITS

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet

#define VARIO_CLIMB_THRESHOLD_CPS_DEFAULT  	50
#define VARIO_CLIMB_THRESHOLD_CPS_MIN   	20
#define VARIO_CLIMB_THRESHOLD_CPS_MAX   	100

#define VARIO_ZERO_THRESHOLD_CPS_DEFAULT  	5
#define VARIO_ZERO_THRESHOLD_CPS_MIN    	-20
#define VARIO_ZERO_THRESHOLD_CPS_MAX    	20

#define VARIO_SINK_THRESHOLD_CPS_DEFAULT  	-250
#define VARIO_SINK_THRESHOLD_CPS_MIN    	-400
#define VARIO_SINK_THRESHOLD_CPS_MAX    	-100


// When generating climbtones, the vario allocates most of the speaker 
// frequency bandwidth to climbrates below this crossover threshold 
// so you have more frequency discrimination. So set the crossover threshold 
// to the average thermal core climbrate you expect for the site and conditions.
#define VARIO_CROSSOVER_CPS_DEFAULT     400
#define VARIO_CROSSOVER_CPS_MIN         300
#define VARIO_CROSSOVER_CPS_MAX         800

// Kalman filter configuration
#define KF_ACCEL_VARIANCE_DEFAULT     100
#define KF_ACCEL_VARIANCE_MIN         50
#define KF_ACCEL_VARIANCE_MAX         150

#define KF_ZMEAS_VARIANCE_DEFAULT    200
#define KF_ZMEAS_VARIANCE_MIN        100
#define KF_ZMEAS_VARIANCE_MAX        400

// Sleep timeout. The vario will go into sleep mode
// if it does not detect climb or sink rates more than
// SLEEP_THRESHOLD_CPS, for the specified minutes.
// You can only exit  sleep mode by power cycling the unit.
#define SLEEP_TIMEOUT_MINUTES_DEFAULT   15
#define SLEEP_TIMEOUT_MINUTES_MIN       5
#define SLEEP_TIMEOUT_MINUTES_MAX       30

// audio feedback tones
#define BATTERY_TONE_HZ       400
#define CALIBRATING_TONE_HZ   800
#define UNCALIBRATED_TONE_HZ  2000
#define MPU9250_ERROR_TONE_HZ   200 
#define MS5611_ERROR_TONE_HZ    2500


#define BLUETOOTH_DEFAULT  1

///////////////////////////////////////////////////////////////////////////////
// COMPILED CONFIGURATION PARAMETERS ( cannot be changed with web configuration )

// change these parameters based on the frequency bandwidth of the speaker
#define VARIO_SPKR_MIN_FREQHZ      	200
#define VARIO_SPKR_MAX_FREQHZ       3200

// three octaves (2:1) of frequency for climbrates below crossoverCps,
// and one octave of frequency for climbrates above crossoverCps.
// This gives you more perceived frequency discrimination for climbrates 
// below crossoverCps
#define VARIO_CROSSOVER_FREQHZ    	1600

// This is set low as the residual acceleration bias after calibration
// is expected to have little variation/drift
#define KF_ACCELBIAS_VARIANCE   0.005f

// KF4 Acceleration Update variance default
#define KF_ACCEL_UPDATE_VARIANCE   50.0f

// any climb/sinkrate excursions beyond this level will keep the
// vario active. If it stays below this level for the configured
// time interval, vario goes to sleep to conserve power
#define SLEEP_THRESHOLD_CPS    50

// if you find that gyro calibration fails even when you leave
// the unit undisturbed, increase this offset limit
// until you find that gyro calibration works consistently.
//#define GYRO_OFFSET_LIMIT_1000DPS   	50
#define GYRO_OFFSET_LIMIT_1000DPS   	50

// pwm settings for lantern brightness
#define LANTERN_DIM   20
#define LANTERN_LOW   50
#define LANTERN_MID   200
#define LANTERN_HI   1023

// print debug information to the serial port for different code modules

#define TOP_DEBUG
#ifdef TOP_DEBUG
	#define dbg_println(x) {Serial.println x;}
	#define dbg_printf(x)  {Serial.printf x;}
#else
	#define dbg_println(x)
	#define dbg_printf(x)
#endif
// these #defines can be left uncommented after debugging, as the enclosed
// debug prints do not appear in the critical run-time loop
#define KF_DEBUG
#define VARIO_DEBUG
#define NVD_DEBUG
#define MPU9250_DEBUG
#define MS5611_DEBUG
#define WEBCFG_DEBUG

// !! ensure these #defines are commented out after debugging, as the 
// enclosed debug prints are in the critical run-time loop.
//#define IMU_DEBUG
//#define CCT_DEBUG


#endif
