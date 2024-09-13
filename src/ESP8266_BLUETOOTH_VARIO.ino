#include <Arduino.h>
#include <Wire.h>
#include <FS.h>
#include <LittleFS.h>
#include <ESP8266mDNS.h>
#include "config.h"
#include "util.h"
#include "imu.h"
#include "mpu9250.h"
#include "ms5611.h"
#include "kalmanfilter4.h"
#include "cct.h"
#include "adc.h"
#include "nvd.h"
#include "audio.h"
#include "ringbuf.h"
#include "ui.h"
#include <Adafruit_NeoPixel.h>
#include "wificfg.h"



int      AppMode;

uint32_t TimePreviousUs;
uint32_t TimeNowUs;
float 	 ImuTimeDeltaUSecs; // time between MPU9250 samples, in microseconds
float 	 KfTimeDeltaUSecs; // time between kalman filter updates, in microseconds

float AccelmG[3]; // in milli-Gs
float GyroDps[3];  // in degrees/second
float KfAltitudeCm = 0.0f; // kalman filtered altitude in cm
float KfClimbrateCps  = 0.0f; // kalman filtered climbrate in cm/s

// pinPCC (GPIO0) has an external 10K pullup resistor to VCC
// pressing the button  will ground the pin.
// This button has three different functions : program, configure, and calibrate (PCC)
// 1. (Program)
//    Power on the unit with PCC button pressed. Or with power on, keep 
//    PCC pressed and momentarily press the reset button.
//    This will put the ESP8266 into programming mode, and you can flash 
//    the application code from the Arduino IDE.
// 2. (WiFi Configuration)
//    After normal power on, immediately press PCC and keep it pressed. 
//    Wait until you hear a low tone, then release. The unit will now be in WiFi configuration
//    configuration mode. 
// 3. (Calibrate)
//    After normal power on, wait until you hear the battery voltage feedback beeps and
//    then the countdown to gyroscope calibration. If you press the PCC button
//    during the gyro calibration countdown, the unit will start accelerometer calibration first. 
//    Accelerometer re-calibration is required if the acceleration calibration values in 
//    flash were never written, or if the entire flash has been erased.

volatile int DrdyCounter;
volatile boolean DrdyFlag;
volatile int SleepCounter;
volatile int BaroCounter;
volatile int SleepTimeoutSecs;

Adafruit_NeoPixel pixels(NUMPIXELS, ledsPin, NEO_GRB + NEO_KHZ800);

boolean bWebConfigure = false;

MPU9250    Mpu9250;
MS5611     Ms5611;

const char* FwRevision = "1.32";

// handles data ready interrupt from MPU9250 (every 2ms)
void IRAM_ATTR drdy_interrupt_handler() {
	DrdyFlag = true;
	DrdyCounter++;
	}	

// setup time markers for Mpu9250, Ms5611 and kalman filter
void time_init() {
	TimeNowUs = TimePreviousUs = micros();
	}

inline void time_update(){
	TimeNowUs = micros();
	ImuTimeDeltaUSecs = TimeNowUs > TimePreviousUs ? (float)(TimeNowUs - TimePreviousUs) : 2000.0f; // if rollover use expected time difference
	TimePreviousUs = TimeNowUs;
	}

void handle_led(int x, int delayTime)  {
	for (int i = 0; i < x; i++) {
		digitalWrite(LED, LOW);   // Włącz diodę
		delay(delayTime);                // Czekaj 300 ms
		digitalWrite(LED, HIGH);    // Wyłącz diodę
		delay(delayTime);                // Czekaj 300 ms
	}
}

void setup_vario() {
	dbg_println(("Vario mode"));
 	Wire.begin(pinSDA, pinSCL);
	Wire.setClock(100000); // set i2c clock frequency to 400kHz, AFTER Wire.begin()
	//Wire.setClock(400000); // set i2c clock frequency to 400kHz, AFTER Wire.begin()
	delay(100);
	pixels.clear();
	pixels.setBrightness(30);
	pixels.fill(0x0000ff00, 0,1); //green color
	pixels.show();
	dbg_println(("\r\nChecking communication with MS5611"));
	if (!Ms5611.read_prom()) {
		dbg_println(("Bad CRC read from MS5611 calibration PROM"));
		Serial.flush();
		ui_indicate_fault_MS5611(); 
		ui_go_to_sleep();   // switch off and then on to fix this
		}
	dbg_println(("MS5611 OK"));
  	pixels.fill(0x0000ff00, 0,2); //green color
	pixels.show();
	dbg_println(("\r\nChecking communication with MPU9250"));
	if (!Mpu9250.check_id()) {
		dbg_println(("Error reading MPU9250 WHO_AM_I register"));
		Serial.flush();
		ui_indicate_fault_MPU9250();
		ui_go_to_sleep();   // switch off and then on to fix this
		}
	dbg_println(("MPU9250 OK"));
	pixels.fill(0x0000ff00, 0,3); //green color
	pixels.show();
    
	DrdyCounter = 0;
	DrdyFlag = false;
	// interrupt output of MPU9250 is configured as push-pull, active high pulse. This is connected to
	// pinDRDYInt (GPIO15) which already has an external 10K pull-down resistor (required for normal ESP8266 boot mode)
	pinMode(pinDRDYInt, INPUT); 
	attachInterrupt(digitalPinToInterrupt(pinDRDYInt), drdy_interrupt_handler, RISING);

	// configure MPU9250 to start generating gyro and accel data  
	Mpu9250.config_accel_gyro();
	pixels.fill(0x0000ff00, 0,4); //green color
	pixels.show();
	// calibrate gyro (and accel if required)
	ui_calibrate_accel_gyro();
	delay(50);  

	pixels.fill(0x0000ff00, 0,5); //green color
	pixels.show();
	  
	dbg_println(("\r\nMS5611 config"));
	Ms5611.reset();
	Ms5611.get_calib_coefficients(); // load MS5611 factory programmed calibration data
	Ms5611.averaged_sample(30); // get an estimate of starting altitude
	Ms5611.init_sample_state_machine(); // start the pressure & temperature sampling cycle

	dbg_println(("\r\nKalmanFilter config"));

	pixels.fill(0x0000ff00, 0,6); //green color
	pixels.show();
	// initialize kalman filter with Ms5611 estimated altitude, and estimated climbrate = 0
	kalmanFilter4_configure((float)Nvd.par.cfg.kf.zMeasVariance, 1000.0f*(float)Nvd.par.cfg.kf.accelVariance, true, Ms5611.altitudeCmAvg, 0.0f, 0.0f);

	pixels.fill(0x0000ff00, 0,7); //green color
	pixels.show();

	time_init();
	KfTimeDeltaUSecs = 0.0f;
	BaroCounter = 0;
	SleepTimeoutSecs = 0;
	ringbuf_init(); 
	SleepCounter = 0;

	dbg_println(("\r\nStarting Vario\r\n"));
	handle_led(3, 300);


	for (int i = 0; i < 3; i++) {
		pixels.fill(0x0000ff00, 0); //green color
		pixels.show();
		delay(400);                // Czekaj 300 ms
		pixels.fill(0, 0); //green color
		pixels.show();
		delay(400);                 // Czekaj 300 ms
	}


	}     


void setup() {
	pinMode(pinPCC, INPUT); //  Program/Configure/Calibrate Button
	pinMode(LED, OUTPUT);    // LED pin as output.


#ifdef TOP_DEBUG    
	Serial.begin(115200);
#endif
  
	dbg_printf(("\r\n\r\nESP8266 BLUETOOTH VARIO compiled on %s at %s\r\n", __DATE__, __TIME__));
	dbg_printf(("Firmware Revision %s\r\n", FwRevision));
	dbg_println(("\r\nChecking non-volatile data (calibration and configuration)"));  
	nvd_init();

	if (!LittleFS.begin()){
		dbg_println(("Error mounting LittleFS"));
		ESP.restart();
		}   

	dbg_println(("Pixels begin"));
	pixels.begin();
	pixels.fill(0, 0); //green color
		pixels.show();

	audio_config(pinAudio); 

	//bWebConfigure = true;
	dbg_println(("To start web configuration mode, press and hold the PCC button"));
	dbg_println(("until you hear a low-frequency tone. Then release the button"));
	for (int cnt = 0; cnt < 4; cnt++) {
		dbg_println((4-cnt));
		delay(1000);
		if (digitalRead(pinPCC) == 0) {
			bWebConfigure = true;
			break;
			}
		}
	if (bWebConfigure == true) {
		dbg_println(("Web configuration mode selected"));
		// 3 second long tone with low frequency to indicate unit is now in web server configuration mode.
		// After you are done with web configuration, switch off the vario as the wifi radio
		// consumes a lot of power.
		pixels.fill(0x00FF33FF, 0); //pink color
		pixels.show();
		wificfg_ap_server_init(); 
		}
  	else {
    	ui_indicate_battery_voltage();

    	switch (AppMode) {
			case APP_MODE_VARIO :
			default :
			setup_vario();
			break;
			}
		}
	ui_btn_init();	
	}

	void update_led_based_on_altitude(float altitudeCm) {
		  static unsigned long lastToggleTime = 0;  // Track the last time LEDs were toggled
    static bool ledState = true;              // LED state for blinking
    unsigned long currentTime = millis();     // Get current time in milliseconds

    // Iterate through all configurations to find the matching altitude range
    for (int i = 0; i < sizeof(configs) / sizeof(configs[0]); i++) {
        if (altitudeCm >= configs[i].minAltitude && altitudeCm < configs[i].maxAltitude) {
            // Set the brightness based on the current configuration
            pixels.setBrightness(configs[i].brightness);

            // Set the LED color based on the current configuration
            if (configs[i].shouldBlink) {
                // If blinking, toggle the LED state based on the blink interval
                if (currentTime - lastToggleTime >= configs[i].blinkInterval) {
                    lastToggleTime = currentTime;
                    ledState = !ledState;  // Toggle the LED state (on/off)
                }
                // Update the LEDs based on the toggled state
                if (ledState) {
                    pixels.fill(configs[i].color, 0);
                } else {
                    pixels.fill(0x00000000, 0); // Turn off LEDs (black)
                }
            } else {
                // If no blinking, just set the LED to the configured color
                pixels.fill(configs[i].color, 0);
            }
            pixels.show(); // Send the updated color and brightness to the LED strip
            return; // Exit the function once the matching range is found
        }
    }

    // If the altitude doesn't match any range, turn off the LEDs
    pixels.setBrightness(0);  // Set brightness to 0 (off)
    pixels.fill(0x00000000, 0);  // Set color to black (off)
    pixels.show();
}


void vario_loop() {
 

	if (DrdyFlag == true) {
		// 500Hz ODR => 2mS sample interval
		DrdyFlag = false;
		time_update();
		#ifdef CCT_DEBUG    
		cct_set_marker(); // set marker for estimating the time taken to read and process the data (needs to be < 2mS !!)
		#endif    
		// accelerometer samples (ax,ay,az) in milli-Gs, gyroscope samples (gx,gy,gz) in degrees/second
		Mpu9250.get_accel_gyro_data(AccelmG, GyroDps); 

		// We arbitrarily decide that in the assembled vario, the CJMCU-117 board silkscreen Y points "forward" or "north",
		// silkscreen X points "right" or "east", and silkscreen Z points down. This is the North-East-Down (NED) 
		// right-handed coordinate frame used in our AHRS algorithm implementation.
		// The required mapping from sensor samples to NED frame for our specific board orientation is : 
		// gxned = gx, gyned = gy, gzned = -gz (clockwise rotations about the axis must result in +ve readings on the sensor axis channel)
		// axned = ay, ayned = ax, azned = az (when the axis points down, sensor axis channel reading must be +ve)
		// The AHRS algorithm expects rotation rates in radians/second
		// Acceleration data is only used for orientation correction when the acceleration magnitude is between 0.75G and 1.25G
		float accelMagnitudeSquared = AccelmG[0]*AccelmG[0] + AccelmG[1]*AccelmG[1] + AccelmG[2]*AccelmG[2];
		int bUseAccel = ((accelMagnitudeSquared > 562500.0f) && (accelMagnitudeSquared < 1562500.0f)) ? 1 : 0;
        float dtIMU = ImuTimeDeltaUSecs/1000000.0f;
        float gxned = DEG_TO_RAD*GyroDps[0];
        float gyned = DEG_TO_RAD*GyroDps[1];
        float gzned = -DEG_TO_RAD*GyroDps[2];
        float axned = AccelmG[1];
        float ayned = AccelmG[0];
        float azned = AccelmG[2];
		imu_mahonyAHRS_update6DOF(bUseAccel, dtIMU, gxned, gyned, gzned, axned, ayned, azned);
		float gCompensatedAccel = imu_gravity_compensated_accel(axned, ayned, azned, Q0, Q1, Q2, Q3);
		ringbuf_add_sample(gCompensatedAccel);  
		int32_t audioCps; // filtered climbrate, rounded to nearest cm/s

		BaroCounter++;
		KfTimeDeltaUSecs += ImuTimeDeltaUSecs;
		if (BaroCounter >= 5) { // 5*2mS = 10mS elapsed, this is the sampling period for MS5611, 
			BaroCounter = 0;    // alternating between pressure and temperature samples
			// one altitude sample is calculated for every new pair of pressure & temperature samples
			int zMeasurementAvailable = Ms5611.sample_state_machine(); 
			if ( zMeasurementAvailable ) { 
				// average earth-z acceleration over the 20mS interval between z samples
				// is used in the kf algorithm update phase
				float zAccelAverage = ringbuf_average_newest_samples(10); 
				float dtKF = KfTimeDeltaUSecs/1000000.0f;
				kalmanFilter4_predict(dtKF);
				kalmanFilter4_update(Ms5611.relativeAltitudeCm, zAccelAverage, (float*)&KfAltitudeCm, (float*)&KfClimbrateCps);
				// reset time elapsed between kalman filter algorithm updates
				KfTimeDeltaUSecs = 0.0f;
				audioCps =  KfClimbrateCps >= 0.0f ? (int32_t)(KfClimbrateCps+0.5f) : (int32_t)(KfClimbrateCps-0.5f);

				
				update_led_based_on_altitude(KfAltitudeCm);

				if (ABS(audioCps) > SLEEP_THRESHOLD_CPS) { 
					// reset sleep timeout watchdog if there is significant vertical motion
					SleepTimeoutSecs = 0;
					}
				else
				if (SleepTimeoutSecs >= (Nvd.par.cfg.misc.sleepTimeoutMinutes*60)) {
					dbg_println(("Timed out with no significant climb/sink, put MPU9250 and ESP8266 to sleep to minimize current draw"));
					Serial.flush();
					ui_indicate_sleep(); 
					ui_go_to_sleep();
					}   
				}
			}
			
	#ifdef CCT_DEBUG      
		uint32_t elapsedUs =  cct_get_elapsedUs(); // calculate time  taken to read and process the data, must be less than 2mS
	#endif
		if (DrdyCounter >= 50) {
			DrdyCounter = 0; // 0.1 second elapsed

			SleepCounter++;
			if (SleepCounter >= 10) {
				SleepCounter = 0;
				SleepTimeoutSecs++;
				
				#ifdef IMU_DEBUG
				float yaw, pitch, roll;
				imu_quaternion_to_yaw_pitch_roll(Q0,Q1,Q2,Q3, &yaw, &pitch, &roll);
				// Pitch is positive for clockwise rotation about the +Y axis
				// Roll is positive for clockwise rotation about the +X axis
				// Yaw is positive for clockwise rotation about the +Z axis
				// Magnetometer isn't used, so yaw is initialized to 0 for the "forward" direction of the case on power up.
				dbg_printf(("\r\nY = %d P = %d R = %d\r\n", (int)yaw, (int)pitch, (int)roll));
				dbg_printf(("ba = %d ka = %d kv = %d\r\n",(int)Ms5611.altitudeCm, (int)KfAltitudeCm, (int)KfClimbrateCps));
				dbg_printf(("altitude kalman = %d\r\n",(int)KfAltitudeCm));
				dbg_printf(("relative altitude = %d\r\n",(int)Ms5611.relativeAltitudeCm));

				#endif

				#ifdef CCT_DEBUG      
                // The raw IMU data rate is 500Hz, i.e. 2000uS between Data Ready Interrupts
                // We need to read the MPU9250 data, MS5611 data and finish all computations
                // and actions well within this interval.
                // Checked, < 620 uS @ 80MHz clock
				dbg_printf(("Elapsed %dus\r\n", (int)elapsedUs)); 
				#endif
				}
			}
    	}	
	}



void loop(){
	if (bWebConfigure == true) {
		MDNS.update();
		}
	else { 
		switch (AppMode) {
			case APP_MODE_VARIO :
			default :
			vario_loop();
			break;    
			}
		} 
	}
