/*
 * This project uses an Arduino Nano (or clone) to control servos that will follow the brightest light source.
 * A common usage for this is to track the sun as it moves across the sky, and direct a solar panel towards it.
 * Use I2C to control a PCA9685 board, which will control the servos.
 * Photocell usage gleaned from: https://learn.adafruit.com/photocells
 * Schematic on this page: https://learn.adafruit.com/photocells/using-a-photocell
 * Use A0 through A3 to read voltage levels.
 * Mapping of the photoresistors to analog pins:
 *    A0 = top left
 *    A1 = top right
 *    A2 = bottom left
 *    A3 = bottom right
 *
 * Photoresistors have a higher resistance in darkness, and a lower resistance in direct light.
 * The batch I bought should be 300 kΩ (kilohm) in direct sunlight, and over 20 MΩ (megohm) in complete darkness.
 * The photoresistor is half of a voltage divider, along with a 10 kΩ resistor.
 * This means most of the voltage will drop over the photoresistor, and only a fraction will drop over the 10 kΩ resistor.
 * The analog inputs will be measuring the voltage drop across the 10 kΩ resistor.
 *
 * Power:
 * Small servos draw about 100 mA when moving under no load, and can draw up to 800 mA when stalled.  So provide about least 1.6 amps just for the servos.
 *
 * Pseudocode:
 * If the sum of the top row (A0 and A1) is greater than the sum of the bottom row (A2 and A3), move down.
 * If the sum of the top row (A0 and A1) is lesser than the sum of the bottom row (A2 and A3), move up.
 * If the sum of the left column (A0 and A2) is greater than the sum of the right column (A1 and A3), move right.
 * If the sum of the left column (A0 and A2) is lesser than the sum of the right column (A1 and A3), move left.
 *
 * Use a cool-down time or "proximity to other value" to keep from jittering the servos nonstop.
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define SERVO_FREQ 50									// Analog servos run at ~50 Hz updates
const uint8_t AZIMUTH_SERVO = 0;						// The azimuth (rotation) servo will use port 0.
const uint8_t ELEVATION_SERVO = 1;					// The elevation servo will use port 1.
const int AZIMUTH_PWM_MIN = 150;						// This sets the leftmost position for the azimuth servo.  100 is too low, and binds.
const int AZIMUTH_PWM_MAX = 450;						// This sets the rightmost position for the azimuth servo.  500 is too high, and causes the servo to "seek" even after moving to its final position.
const int AZIMUTH_STEP_SIZE = 1;						// The maximum PWM change per step.  A smaller values results in smoother motion.
const int ELEVATION_PWM_MIN = 250;					// This sets the lowest declination for the elevation servo.
const int ELEVATION_PWM_MAX = 360;					// This sets the highest inclination for the elevation servo.
const int ELEVATION_STEP_SIZE = 1;					// The maximum PWM change per step.  A smaller values results in smoother motion.
const int TOP_LEFT_PHOTOCELL = 0;					// The top-left photocell will be read by analog GPIO 0.
const int TOP_RIGHT_PHOTOCELL = 1;					// The top-right photocell will be read by analog GPIO 1.
const int BOTTOM_LEFT_PHOTOCELL = 2;				// The bottom-left photocell will be read by analog GPIO 2.
const int BOTTOM_RIGHT_PHOTOCELL = 3;				// The bottom-right photocell will be read by analog GPIO 3.
const unsigned long SERIAL_PRINT_DELAY = 2000;	// This is the delay between prints to the serial monitor.
const unsigned long SERVO_MOVE_DELAY = 10;		// The minimum time, in millisecond, between servo movements.
const int LIGHT_RANGE = 80;							// The value a delta must be greater than to consider moving a servo.  This prevents "jitter" when the tracker is in.

uint16_t azimuthPulseWidth = AZIMUTH_PWM_MIN + ( ( AZIMUTH_PWM_MAX - AZIMUTH_PWM_MIN ) / 2 );			// This should be the center position.
uint16_t elevationPulseWidth = ELEVATION_PWM_MIN + ( ( ELEVATION_PWM_MAX - ELEVATION_PWM_MIN ) / 2 );	// This should be the center position.
int topLeftReading;
int topRightReading;
int bottomLeftReading;
int bottomRightReading;
int topSum;
int bottomSum;
int leftSum;
int rightSum;
int elevationDelta = 0;
int azimuthDelta = 0;
unsigned long lastSerialPrintTime = 0;				// Used to track the time of the last output to the serial monitor.
unsigned long lastServoMoveTime = 0;				// Used to track the time of the last servo move.


void setup()
{
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );

	Serial.println( "" );
	Serial.println( "setup()" );
	Serial.println( __FILE__ );

	pwm.begin();
	pwm.setOscillatorFrequency( 27000000 );
	pwm.setPWMFreq( SERVO_FREQ );  // Analog servos run at ~50 Hz updates

	// Set the azimuth servo to its starting position.
	pwm.setPWM( AZIMUTH_SERVO, 0, azimuthPulseWidth );
	pwm.setPWM( ELEVATION_SERVO, 0, elevationPulseWidth );

	printStats();

	// Initialize the LED.
	pinMode( LED_BUILTIN, OUTPUT );
	// Turn the LED on to show setup() is beginning.
	digitalWrite( LED_BUILTIN, HIGH );
	delay( 1000 );
	// Turn the LED off until setup() finishes.
	digitalWrite( LED_BUILTIN, LOW );

	Serial.println( "setup() complete!" );
	Serial.println( "" );
	// Turn the LED on to show that setup() is complete.
	digitalWrite( LED_BUILTIN, HIGH );
} // End of setup() function.


void printStats()
{
	Serial.println( "" );
	Serial.println( "Readings:" );
	Serial.print( topLeftReading );
	Serial.print( " | " );
	Serial.println( topRightReading );
	Serial.println( "---------" );
	Serial.print( bottomLeftReading );
	Serial.print( " | " );
	Serial.println( bottomRightReading );
	Serial.println( "" );

	Serial.println( "topSum | bottomSum:" );
	Serial.println( topSum );
	Serial.println( "---" );
	Serial.println( bottomSum );
	Serial.println( "" );
	Serial.println( "leftSum | rightSum:" );
	Serial.print( leftSum );
	Serial.print( " | " );
	Serial.println( rightSum );
	Serial.println( "" );

	Serial.print( "Azimuth pulse width:   " );
	Serial.print( AZIMUTH_PWM_MIN );
	Serial.print( " - " );
	Serial.print( azimuthPulseWidth );
	Serial.print( " - " );
	Serial.println( AZIMUTH_PWM_MAX );
	Serial.print( "Azimuth delta:   " );
	Serial.println( azimuthDelta );

	Serial.print( "Elevation pulse width: " );
	Serial.print( ELEVATION_PWM_MIN );
	Serial.print( " - " );
	Serial.print( elevationPulseWidth );
	Serial.print( " - " );
	Serial.println( ELEVATION_PWM_MAX );
	Serial.print( "Elevation delta: " );
	Serial.println( elevationDelta );

	if( abs( topSum - bottomSum ) > LIGHT_RANGE )
	{
		if( topSum > bottomSum )
			Serial.println( "Move up" );
		if( bottomSum > topSum )
			Serial.println( "Move down" );
	}
	else
		Serial.println( "No elevation change needed." );
	if( abs( leftSum - rightSum ) > LIGHT_RANGE )
	{
		if( leftSum > rightSum )
			Serial.println( "Move left" );
		if( rightSum > leftSum )
			Serial.println( "Move right" );
	}
	else
		Serial.println( "No azimuth change needed." );
} // End of printStats() function.


void moveServosIncrementally()
{
	if( abs( leftSum - rightSum ) > LIGHT_RANGE )
	{
		if( leftSum > rightSum )
		{
			moveLeft();
		}
		if( rightSum > leftSum )
		{
			moveRight();
		}
		pwm.setPWM( AZIMUTH_SERVO, 0, azimuthPulseWidth );
	}
	if( abs( topSum - bottomSum ) > ( LIGHT_RANGE * 2 ) )
	{
		if( topSum > bottomSum )
			moveUp();
		if( bottomSum > topSum )
			moveDown();
		pwm.setPWM( ELEVATION_SERVO, 0, elevationPulseWidth );
	}
} // End of moveServosIncrementally() function.


void moveLeft()
{
	if( azimuthPulseWidth - AZIMUTH_STEP_SIZE > AZIMUTH_PWM_MIN )
	{
		azimuthPulseWidth -= AZIMUTH_STEP_SIZE;
	}
	// else
		// Serial.println( "Limits prevent moving farther left!" );
} // End of moveLeft() function.


void moveRight()
{
	if( azimuthPulseWidth + AZIMUTH_STEP_SIZE < AZIMUTH_PWM_MAX )
	{
		azimuthPulseWidth += AZIMUTH_STEP_SIZE;
	}
	// else
		// Serial.println( "Limits prevent moving farther right!" );
} // End of moveRight() function.


void moveUp()
{
	if( elevationPulseWidth + ELEVATION_STEP_SIZE < ELEVATION_PWM_MAX )
	{
		elevationPulseWidth += ELEVATION_STEP_SIZE;
	}
	// else
		// Serial.println( "Limits prevent moving farther up!" );
} // End of moveUp() function.


void moveDown()
{
	if( elevationPulseWidth - ELEVATION_STEP_SIZE > ELEVATION_PWM_MIN )
	{
		elevationPulseWidth -= ELEVATION_STEP_SIZE;
	}
	// else
		// Serial.println( "Limits prevent moving farther down!" );
} // End of moveDown() function.


void setAzimuthAbsolutePWM( uint16_t pulseWidth )
{
	Serial.println( "Setting azimuth servo to minimum (full-left)..." );
	for( uint16_t pulseLength = AZIMUTH_PWM_MAX; pulseLength > AZIMUTH_PWM_MIN; pulseLength-- )
	{
		pwm.setPWM( AZIMUTH_SERVO, 0, pulseWidth );
		Serial.print( pwm.getPWM( 0 ) );
	}
	Serial.println( "\nDone!" );
} // End of setAzimuthAbsolutePWM() function.


void setElevationAbsolutePWM( uint16_t pulseWidth )
{
	Serial.println( "Setting azimuth servo to maximum (full-right)..." );
	// pwm.setPWM( AZIMUTH_SERVO, 0, AZIMUTH_PWM_MAX );
	for( uint16_t pulseLength = AZIMUTH_PWM_MIN; pulseLength < AZIMUTH_PWM_MAX; pulseLength++ )
	{
		pwm.setPWM( AZIMUTH_SERVO, 0, pulseLength );
		Serial.print( pwm.getPWM( 0 ) );
	}
	Serial.println( "\nDone!" );
} // End of setElevationAbsolutePWM() function.


void loop()
{
	unsigned long time;

	// Read all photocell values.
	topLeftReading = analogRead( TOP_LEFT_PHOTOCELL );
	topRightReading = analogRead( TOP_RIGHT_PHOTOCELL );
	bottomLeftReading = analogRead( BOTTOM_LEFT_PHOTOCELL );
	bottomRightReading = analogRead( BOTTOM_RIGHT_PHOTOCELL );

	// Sum each side.
	topSum = topLeftReading + topRightReading;
	bottomSum = bottomLeftReading + bottomRightReading;
	leftSum = topLeftReading + bottomLeftReading;
	rightSum = topRightReading + bottomRightReading;

	// Deltas
	elevationDelta = topSum - bottomSum;
	azimuthDelta = leftSum - rightSum;

	// Map 0-1023 from analogRead() to 0-180 for the servo.
	// elevationPulseWidth = map( elevationDelta, 1023, -1023, ELEVATION_PWM_MIN, ELEVATION_PWM_MAX );
	// azimuthPulseWidth = map( azimuthDelta, -1023, 1023, AZIMUTH_PWM_MIN, AZIMUTH_PWM_MAX );

	time = millis();
	if( lastServoMoveTime == 0 || ( ( time > SERVO_MOVE_DELAY ) && ( time - SERVO_MOVE_DELAY ) > lastServoMoveTime ) )
	{
		moveServosIncrementally();
		lastServoMoveTime = millis();
	}

	time = millis();
	if( lastSerialPrintTime == 0 || ( ( time > SERIAL_PRINT_DELAY ) && ( time - SERIAL_PRINT_DELAY ) > lastSerialPrintTime ) )
	{
		Serial.println( "" );
		printStats();
		Serial.println( "" );
		lastSerialPrintTime = millis();
	}
} // End of loop() function.
