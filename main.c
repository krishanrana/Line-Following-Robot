/* ========== EGB220 Semester 1 2016 Team 11 Line following robot code ========== */

// ---------- Includes --------- //
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdbool.h>

// ---------- Useful macros --------- //
#define readBit(val, bit) (val & (1<<bit))
#define setBit(val, bit) (val = (val | (1<<bit)))
#define clearBit(val, bit) (val = (val & ~(1<<bit)))

// --------- Defines ---------- //
#define SENSOR_ADC_1 7 // The ADC number that the sensor is connected to
#define SENSOR_ADC_2 6
#define L_SENSOR_ADC 10
#define R_SENSOR_ADC 4
#define R_ENCODER_PORT PORTC
#define R_ENCODER_PIN 6
#define L_ENCODER_PORT PORTD
#define L_ENCODER_PIN 4

#define LEFT_MOTOR OCR0B // PWM output compare register A, controls PWM output A, connected to the left motor
#define LEFT_MOTOR_NEG OCR0A
#define RIGHT_MOTOR_NEG OCR1A // PWM output compare register B, controls PWM output B, connected to the right motor
#define RIGHT_MOTOR OCR1B

#define BLACK_10BIT 700 // Values above this are still black; vlaues less than this grey
#define WHITE_10BIT 330 // Values below this are still white; values greater than this are grey

#define N_ADCS 4

// --------- Globals ---------- //
uint8_t adc_list[] = { SENSOR_ADC_1, SENSOR_ADC_2, L_SENSOR_ADC, R_SENSOR_ADC }; // ADC numbers
uint16_t adc_values[N_ADCS]; // 10 bit ADC values after reading

volatile float position = 0; // Position read from the front sensors
volatile float derivative = 0; // Rate of change of position
volatile uint16_t corner_integral_counter = 0;
volatile uint32_t r_encoder_ticks = 0; // Number of ticks read by the right encoder since program start
volatile uint16_t r_velocity = 0; // Velocity of the right encoder (mm/0.1s)
volatile uint32_t l_encoder_ticks = 0;
volatile uint16_t l_velocity = 0;
uint8_t speed = 0; // Speed value passed into the drive function
int16_t turning_hists = 0; // Used to determine if on a corner
uint8_t lap_count = 0; // Number of times a lap has been compelted
uint32_t speedzone_ticks = 0; // Number of tick since the beggining of the lap the speedzone is located
uint16_t green_calibration = 0; // Calibration value for the green speedzone marker


// --------- Function Declarations ---------- //
void setupADC(void);
void setupTimer0(void); // Sets up 8 bit Timer 0 for PWM and encoder interrupt
void setupTimer1(void); // Sets up 16 bit Timer 1 for PWM in 8 bit mode
void setupTimer3(void); // Sets up Timer 3 for the system time
uint32_t getSystemTime(void); // Retuns the time since program start in 0.1ms increments
void drive(float direction, uint8_t speed); // Controls the motor speeds accroding to the direction and speed
void readEncoders(void); // Reads the encoders storing the velocity and ticks in globals
void checkStartStop(void); // Checks for the start and stop markers
bool checkGreen(void); // Checks for the beggining of the speedzone


ISR(ADC_vect) { // Fired whenever an ADC read completes
	volatile static uint8_t current_adc_index = 0; // The index of adc_list and adc_values being read
	volatile static float direction = 0; // To keep track of which side of the line it's on if it goes off the line
	volatile static float last_position = 0; // To calculate the derivative
	adc_values[current_adc_index] = ADCL | (ADCH << 8); // Store the previous reading
	
	current_adc_index++; // Increment to the next reading
	if (current_adc_index >= N_ADCS) { // If all the ADCs have been read
		current_adc_index = 0; // Reset so the ADCs can be read again
		position = (float)((int16_t)adc_values[1]-(int16_t)adc_values[0])/400.f; // Compute the difference and scale so always between -2 and 2
		
		
		if ((adc_values[0] > BLACK_10BIT) & (adc_values[1] > BLACK_10BIT)) { // If both are reading black (off the track)
			if (direction > 0) { // If on the right
				position = 2; // Set fully right
			} else {
				position = -2; // Set fully left
			}
		} else if ((position > 0.5)) { // Slightly to the right
			direction = 1; // Moving to the right
		} else if ((position < -0.5)) { // Slightly to the left
			direction = -1; // Moving to the left
		}
		derivative = position - (last_position); // Derivative is simply the difference between this reading and the last
		last_position = position; // Record the position for the next derivative
	}
	
	ADMUX = ADMUX & 0b11100000; // Reset the ADC select (MUX) bits
	if (adc_list[current_adc_index] <= 7) { // If <= 7 MUX5 must be set to 0
		clearBit(ADCSRB, MUX5); // MUX5 is in ADCSRB instead of ADMUX
		ADMUX = ADMUX | adc_list[current_adc_index]; // Set the ADC number
	} else {
		setBit(ADCSRB, MUX5);
		ADMUX = ADMUX | (adc_list[current_adc_index] & 0b00000111); // We only want the last 3 bits as we have already dealt with MUX5
	}
	ADCSRA = ADCSRA | (1 << ADSC); // Start conversion
}


ISR(TIMER0_OVF_vect) { // Encoder interrupt
	readEncoders(); // This must be run at a high enough frequency
	checkStartStop(); // Only check start stop when the encoders have been read
}


void setupTimer0(void) { // Sets up timer 0 for PWM and overflow interrupt for encoders
	setBit(TCCR0B, CS01); // Timer prescaler of 8, 7.8125kHz
	
	// Fast PWM mode:
	setBit(TCCR0A, WGM00);
	setBit(TCCR0A, WGM01);
	clearBit(TCCR0A, WGM02);
	
	// Enable output A (Clear on compare match, high on bottom)
	setBit(TCCR0A, COM0A1);
	clearBit(TCCR0A, COM0A0);
	// Enable output B (Clear on compare match, high on bottom)
	setBit(TCCR0A, COM0B1);
	clearBit(TCCR0A, COM0B0);
    
	// Pins as outputs:
    setBit(DDRD, 0);
    setBit(DDRB, 7);
    
	setBit(TIMSK0, TOIE0); // Enable interrupt for encoder
}


void setupTimer1(void) {
	setBit(TCCR1B, CS11);  // Prescaler of 8
	
	// Fast PWM mode, 8 bit:
	setBit(TCCR1A, WGM10); // Bit 0
	setBit(TCCR1B, WGM12); // Bit 3
	
	// Clear on compare match, set at TOP (for fast PWM) (set COM1n1 for each output)
	setBit(TCCR1A, COM1A1);
 	setBit(TCCR1A, COM1B1); // Bit 5
	
	// Pins as outputs:
	setBit(DDRB, 6);
	setBit(DDRB, 5);
} 


void setupTimer3(void) { // Sets up timer 3 for the system time
	// Prescaler (1024)
	setBit(TCCR3B, CS32);
	clearBit(TCCR3B, CS31);
	setBit(TCCR3B, CS30);

	// Normal mode
	clearBit(TCCR3A, WGM30);
	clearBit(TCCR3A, WGM31);
	clearBit(TCCR3B, WGM32);
	clearBit(TCCR3B, WGM33);
}


uint32_t getSystemTime(void) { // Returns time since MCU start in ms/10
	static uint32_t timer_ovf_count = 0; // Number of times the counter has overflowed
	static uint16_t prev_value = 0; // Previous counter value

	if(TCNT3 < prev_value) { // If overflowed (we could use the OVF interrupt instead)
		timer_ovf_count++; // Increment overflow counter
	}

	prev_value = TCNT3; // Store for next time
	return (((uint32_t)TCNT3*10)/(16)) + 40690*timer_ovf_count; // Time formula
}


/* Each ADC conversion takes only 13 ADC clock cycles. (13*ADC_Prescaler clock cycles)
ADC_Freq = F_CPU/ADC_Prescaler
Max ADC freq is 200 kHz
ADHSM bit can be set for higher frequency but higher power consumption.
16 MHz / 128 = 125 kHz
Using a prescaler of 128, each conversion takes 13*128 = 1664 clock cycles,
or 0.104 ms, or 9615 Hz
*/
void setupADC(void) {
	ADCSRA = ADCSRA | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // set the clock prescaler
	setBit(ADCSRA, ADEN); // Enable the ADC
	setBit(ADCSRA, ADIE); // Enable ADC interrupt
	setBit(ADMUX, REFS0); // Reference voltage
	sei(); // Enable global interrupts
	ADCSRA = ADCSRA | (1 << ADSC); // Start conversion
}


void drive(float direction, uint8_t speed) { // Receives direction and speed as inputs to control motors
	// Speed is between 0 and 255, direction is between -2 and 2
	int8_t int_dir = ceil(direction); // To avoid really slow floating point comparisons
	if (int_dir >= 2) { // Hard right
		LEFT_MOTOR = speed;
		LEFT_MOTOR_NEG = 0;
		RIGHT_MOTOR = 0;
		RIGHT_MOTOR_NEG = speed*(direction-1);
    } else if (int_dir <= -1) { // Hard left
		RIGHT_MOTOR = speed;
		RIGHT_MOTOR_NEG = 0;
		LEFT_MOTOR = 0;
		LEFT_MOTOR_NEG = speed*(-direction-1);
	} else if (int_dir == 1) { // Right
        LEFT_MOTOR = speed;
		LEFT_MOTOR_NEG = 0;
        RIGHT_MOTOR = (uint8_t) speed-(speed*direction);
		RIGHT_MOTOR_NEG = 0;
    } else if (int_dir == -0) { // Left
        RIGHT_MOTOR = speed;
		RIGHT_MOTOR_NEG = 0;
        LEFT_MOTOR = (uint8_t) speed-(speed*(-direction));
		LEFT_MOTOR_NEG = 0;
    } else {
        LEFT_MOTOR = speed;
		LEFT_MOTOR_NEG = 0;
        RIGHT_MOTOR = speed;
		RIGHT_MOTOR_NEG = 0;
    }
}


void readEncoders(void) { // Reads both encoders and stores velocity and ticks in globals
	static uint8_t r_last_encoder_value = 0;
	static uint32_t r_last_time = 0;
	static uint8_t l_last_encoder_value = 0;
	static uint32_t l_last_time = 0;
	// Read encoders:
	uint8_t r_encoder_value = readBit(PINC, R_ENCODER_PIN);
	uint8_t l_encoder_value = readBit(PIND, L_ENCODER_PIN);

	// ----- RIGHT ----- //
	if((r_encoder_value) && !(r_last_encoder_value)){
		uint32_t current_time = getSystemTime();
		r_encoder_ticks++;
		r_velocity = 20000/(current_time-r_last_time);
		r_last_time = current_time;
	}
	if(!(r_encoder_value) && (r_last_encoder_value)){
		r_encoder_ticks ++;
	}
	r_last_encoder_value = r_encoder_value;
	
	// ----- LEFT ----- //
	if((l_encoder_value) && !(l_last_encoder_value)){
		uint32_t current_time = getSystemTime();
		l_encoder_ticks++;
		l_velocity = 20000/(current_time-l_last_time);
		l_last_time = current_time;
	}
	if(!(l_encoder_value) && (l_last_encoder_value)){
		l_encoder_ticks ++;
	}
	l_last_encoder_value = l_encoder_value;
	
	// Calculate turning integral:
	turning_hists = turning_hists + (abs(r_velocity-l_velocity)/9)-7;
	if (turning_hists >= 4800) turning_hists = 4800;
	if (turning_hists <= -4800) turning_hists = -4800;
}


void checkStartStop(void) { // Checks left side sensors for start and stop markers
	static uint32_t line_counter = 0; // Counter to ensure only reading if the encoder is incremented
	static uint8_t on_line = 0; // Bool for if on the line
	static uint8_t first = 1; // If reading the first marker
	
	if (r_encoder_ticks >= (line_counter+1)) { // If one encoder tick has passed since last reading
		line_counter = r_encoder_ticks; // Store for next time
		if ((adc_values[3] <= 620) && (adc_values[2] >= 450)) { // If both sensors are within the threshold
			on_line++; // Increment on_line counter
		} else {
			on_line = 0; // If not on a line, reset the counter
		}
	}
	if (on_line >= 3) { // If the last 3 readings have been positive
		if (first) { // If the first (start) line is read
			first = 0; // No longer first
			on_line = 0; // Reset
			r_encoder_ticks = 0; // Reset the encoder ticks, setting this as the start of the track
			l_encoder_ticks = 0;
		} else if (r_encoder_ticks >= 200) { // If passed a reasonable distance since first line
			// Pause and start again:
			LEFT_MOTOR_NEG = 70; // Set to reverse for a short time to stop
			RIGHT_MOTOR_NEG = 70;
			RIGHT_MOTOR = 0;
			LEFT_MOTOR = 0;
			_delay_ms(250);
			LEFT_MOTOR_NEG = 0;
			RIGHT_MOTOR_NEG = 0;
			_delay_ms(3000);
			
			// Reset for next lap:
			r_encoder_ticks = 0;
			l_encoder_ticks = 0;
			line_counter = 0;
			first = 1; // We are expecting the start marker soon
			lap_count++; // Increment lap count
		}
	}
}


bool checkGreen(void) {
	static uint8_t on_line = 0; // Same method as checkStartStop
	static uint32_t line_counter = 0;
	uint8_t first = 1;
	if (r_encoder_ticks >= (line_counter+1)) {
		line_counter = r_encoder_ticks;
		if ((adc_values[2] <= 500) && (adc_values[2] >= (green_calibration-8))) { // If within the green threshold
			on_line++;
		} else {
			on_line = 0;
		}
	}
	if (r_encoder_ticks >= (line_counter + 500)) first = 1; // If past enough distance tor record a new green marker
	if ((on_line >= 5) && first) {
		on_line = 0;
		first = 0;
		return true;
	}
	return false;
}


int main(void) {
	setupADC(); // Initialize the ADC
	setupTimer0(); // Setup timers
    setupTimer1();
    setupTimer3();
    
	// Set LED pins to output:
    setBit(DDRE, 6); // LED1
    setBit(DDRB, 0); // LED2
	setBit(DDRB, 1); // LED2
	setBit(DDRB, 2); // LED4

	clearBit(DDRC, R_ENCODER_PIN); // Set encoder pins to output
	clearBit(DDRC, L_ENCODER_PIN);

	// Turn of all LEDs:
	clearBit(PORTE, 6); // LED 1
	clearBit(PORTB, 0); // LED 2
	clearBit(PORTB, 1); // LED 3
	clearBit(PORTB, 2); // LED 4
	
	// Get green marker calibration:
	_delay_ms(400);
	green_calibration = adc_values[2];
	_delay_ms(3000);
	
	while(1) { // Loop forever
		static float Kp = 0; // Proportional constant for the PID controller
		
		if(turning_hists >= 1000) { // Corner
			speed = 135; // Slow down
			Kp = 1.f;
		} else { // Straight
			speed = 210;
			Kp = 0.83;
		}
		
		if (checkGreen() && (lap_count == 0)) { // If passed the green marker
			setBit(PORTE, 6); // Turn on LED
			speedzone_ticks = ((r_encoder_ticks+l_encoder_ticks)/2)-200; // Record the speedzone location
		}
		
		uint32_t distance = (r_encoder_ticks+l_encoder_ticks)/2;
		if ((distance >= speedzone_ticks) && (lap_count >= 1) && (distance <= (speedzone_ticks+600))) { // If passed the speedzone
			speed = 118; // Speedzone speed
			Kp = 1.1f;
			setBit(PORTB, 2); // Turn on LED
		} else {
			clearBit(PORTB, 2);
		}
		
		drive((-Kp*position) - (derivative*21.f), speed); // Drive with the set speed and direction PID formula
	}
	return 0;
}