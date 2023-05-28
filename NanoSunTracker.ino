/*
 * This project uses an Arduino Nano (or clone) to control servos that will follow the brightest light source.
 * A common usage for this is to track the sun as it moves across the sky, and direct a solar panel towards it.
 * Photocell usage gleaned from: https://learn.adafruit.com/photocells
 * Schematic on this page: https://learn.adafruit.com/photocells/using-a-photocell
 * Use A0 through A3 to read voltage levels.
 * Mapping of the photoresistors to analog pins:
 *    A0 = top left
 *    A1 = top right
 *    A2 = bottom left
 *    A3 = bottom right
 * Use D3 for the elevation servo and D10 for the azimuth servo.
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
 * Read all 4 analog inputs.
 * If the sum of the top row (A0 and A1) is greater than the sum of the bottom row (A2 and A3), move down.
 * If the sum of the top row (A0 and A1) is lesser than the sum of the bottom row (A2 and A3), move up.
 * If the sum of the left column (A0 and A2) is greater than the sum of the right column (A1 and A3), move right.
 * If the sum of the left column (A0 and A2) is lesser than the sum of the right column (A1 and A3), move left.
 *
 * Use a cool-down time or "proximity to other value" to keep from jittering the servos nonstop.
 */

#include <Servo.h>

#define DEBUG
#define NO_PLOTTER


Servo elevationServo;
Servo azimuthServo;


const int AZIMUTH_MIN = 0;																			  // This sets the leftmost position for the azimuth servo.  This prevents mechanical binding.
const int AZIMUTH_MAX = 180;																		  // This sets the rightmost position for the azimuth servo.  This prevents mechanical binding.
const int AZIMUTH_STEP_SIZE = 1;																	  // The maximum azimuth servo change per step.  A smaller values results in smoother motion.
const int ELEVATION_MIN = 40;																		  // This sets the lowest declination for the elevation servo.  This prevents mechanical binding.
const int ELEVATION_MAX = 130;																	  // This sets the highest inclination for the elevation servo.  This prevents mechanical binding.
const int ELEVATION_STEP_SIZE = 1;																  // The maximum elevation servo change per step.  A smaller values results in smoother motion.
const int TOP_LEFT_PHOTOCELL = 0;																  // The top-left photocell will be read by analog GPIO 0.
const int TOP_RIGHT_PHOTOCELL = 1;																  // The top-right photocell will be read by analog GPIO 1.
const int BOTTOM_LEFT_PHOTOCELL = 2;															  // The bottom-left photocell will be read by analog GPIO 2.
const int BOTTOM_RIGHT_PHOTOCELL = 3;															  // The bottom-right photocell will be read by analog GPIO 3.
const int LIGHT_RANGE = 80;																		  // The value a delta must be greater than to consider moving a servo.  This prevents "jitter" when the tracker is in.
const int TOP_LEFT_CALIBRATE = 0;																  // A value to adjust the top-left reading by.
const int TOP_RIGHT_CALIBRATE = -120;															  // A value to adjust the top-right reading by.
const int BOTTOM_LEFT_CALIBRATE = 0;															  // A value to adjust the bottom-left reading by.
const int BOTTOM_RIGHT_CALIBRATE = 0;															  // A value to adjust the bottom-right reading by.
const unsigned long SERIAL_PRINT_DELAY = 2000;												  // This is the delay between prints to the serial monitor.
const unsigned long SERVO_MOVE_DELAY = 10;													  // The minimum time, in millisecond, between servo movements.

int azimuthPosition = AZIMUTH_MIN + ( ( AZIMUTH_MAX - AZIMUTH_MIN ) / 2 );			  // This should be the center position.
int elevationPosition = ELEVATION_MIN + ( ( ELEVATION_MAX - ELEVATION_MIN ) / 2 ); // This should be the center position.
int topLeftReading = 0;
int topRightReading = 0;
int bottomLeftReading = 0;
int bottomRightReading = 0;
int topSum = 0;
int bottomSum = 0;
int leftSum = 0;
int rightSum = 0;
int elevationDelta = 0;
int azimuthDelta = 0;
unsigned long lastSerialPrintTime = 0; // Used to track the time of the last output to the serial monitor.
unsigned long lastServoMoveTime = 0;	// Used to track the time of the last servo move.


void setup()
{
	Serial.begin( 115200 );
	if( !Serial )
		delay( 1000 );

#ifdef DEBUG
	Serial.println( "" );
	Serial.println( "setup()" );
	Serial.println( __FILE__ );
	printStats();
#endif

	elevationServo.attach( 3 );
	azimuthServo.attach( 10 );

	// Initialize the LED.
	pinMode( LED_BUILTIN, OUTPUT );
	// Turn the LED on to show setup() is beginning.
	digitalWrite( LED_BUILTIN, HIGH );
	delay( 1000 );
	// Turn the LED off until setup() finishes.
	digitalWrite( LED_BUILTIN, LOW );

#ifdef DEBUG
	Serial.println( "setup() complete!" );
	Serial.println( "" );
#endif

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
	Serial.print( AZIMUTH_MIN );
	Serial.print( " - " );
	Serial.print( azimuthPosition );
	Serial.print( " - " );
	Serial.println( AZIMUTH_MAX );
	Serial.print( "Azimuth delta:   " );
	Serial.println( azimuthDelta );

	Serial.print( "Elevation pulse width: " );
	Serial.print( ELEVATION_MIN );
	Serial.print( " - " );
	Serial.print( elevationPosition );
	Serial.print( " - " );
	Serial.println( ELEVATION_MAX );
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


void moveLeft()
{
	if( azimuthPosition - AZIMUTH_STEP_SIZE > AZIMUTH_MIN )
	{
		azimuthPosition -= AZIMUTH_STEP_SIZE;
	}
	// else
	// Serial.println( "Limits prevent moving farther left!" );
} // End of moveLeft() function.


void moveRight()
{
	if( azimuthPosition + AZIMUTH_STEP_SIZE < AZIMUTH_MAX )
	{
		azimuthPosition += AZIMUTH_STEP_SIZE;
	}
	// else
	// Serial.println( "Limits prevent moving farther right!" );
} // End of moveRight() function.


void moveUp()
{
	if( elevationPosition + ELEVATION_STEP_SIZE < ELEVATION_MAX )
	{
		elevationPosition += ELEVATION_STEP_SIZE;
	}
	// else
	// Serial.println( "Limits prevent moving farther up!" );
} // End of moveUp() function.


void moveDown()
{
	if( elevationPosition - ELEVATION_STEP_SIZE > ELEVATION_MIN )
	{
		elevationPosition -= ELEVATION_STEP_SIZE;
	}
	// else
	// Serial.println( "Limits prevent moving farther down!" );
} // End of moveDown() function.


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
		azimuthServo.write( azimuthPosition );
	}
	if( abs( topSum - bottomSum ) > ( LIGHT_RANGE * 2 ) )
	{
		if( topSum > bottomSum )
			moveUp();
		if( bottomSum > topSum )
			moveDown();
		elevationServo.write( elevationPosition );
	}
} // End of moveServosIncrementally() function.


void loop()
{
	unsigned long time;

	// Read all photocell values.
	topLeftReading = analogRead( TOP_LEFT_PHOTOCELL );
	topRightReading = analogRead( TOP_RIGHT_PHOTOCELL );
	bottomLeftReading = analogRead( BOTTOM_LEFT_PHOTOCELL );
	bottomRightReading = analogRead( BOTTOM_RIGHT_PHOTOCELL );

	topLeftReading += TOP_LEFT_CALIBRATE;
	topRightReading += TOP_RIGHT_CALIBRATE;
	bottomLeftReading += BOTTOM_LEFT_CALIBRATE;
	bottomRightReading += BOTTOM_RIGHT_CALIBRATE;

	// Sum each side.
	topSum = topLeftReading + topRightReading;
	bottomSum = bottomLeftReading + bottomRightReading;
	leftSum = topLeftReading + bottomLeftReading;
	rightSum = topRightReading + bottomRightReading;

	// Deltas
	elevationDelta = topSum - bottomSum;
	azimuthDelta = leftSum - rightSum;

	// Map 0-1023 from analogRead() to 0-180 for the servo.
	// elevationPosition = map( elevationDelta, 1023, -1023, ELEVATION_PWM_MIN, ELEVATION_PWM_MAX );
	// azimuthPosition = map( azimuthDelta, -1023, 1023, AZIMUTH_PWM_MIN, AZIMUTH_PWM_MAX );

	time = millis();
	if( lastServoMoveTime == 0 || ( ( time > SERVO_MOVE_DELAY ) && ( time - SERVO_MOVE_DELAY ) > lastServoMoveTime ) )
	{
		moveServosIncrementally();
		lastServoMoveTime = millis();
	}

	time = millis();
	if( lastSerialPrintTime == 0 || ( ( time > SERIAL_PRINT_DELAY ) && ( time - SERIAL_PRINT_DELAY ) > lastSerialPrintTime ) )
	{
#ifdef DEBUG
		Serial.println( "" );
		printStats();
		Serial.println( "" );
#endif
		lastSerialPrintTime = millis();
	}
#ifdef PLOTTER
	//	Serial.print( "Ceiling:" );
	//	Serial.print( 1023 );
	//	Serial.print( "," );
	Serial.print( "Upper-left:" );
	Serial.print( topLeftReading );
	Serial.print( "," );
	Serial.print( "Upper-right:" );
	Serial.print( topRightReading );
	Serial.print( "," );
	Serial.print( "Lower-left:" );
	Serial.print( bottomLeftReading );
	Serial.print( "," );
	Serial.print( "Lower-right:" );
	Serial.print( bottomRightReading );
	Serial.print( "," );
	//	Serial.print( "Floor:" );
	//	Serial.print( 0 );
	//	Serial.print( "," );
	Serial.print( "Elevation:" );
	Serial.print( elevationPosition );
	Serial.print( "," );
	Serial.print( "Azimuth:" );
	Serial.print( azimuthPosition );
	Serial.println();
#endif
} // End of loop() function.
