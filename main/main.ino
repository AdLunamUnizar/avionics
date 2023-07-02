#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"

#define minAltitudeToDeployParachute 1 // 20 meters minimum to deploy parachute

// GLOBAL VARIABLES ------------------------------------------------------------
// Acceleration measurements
// float x, y, z;

// Pressure measurements
float initialAltitude = 0.0f, lastAltitude = 0.0f, currentAltitude = 0.0f;
float initialPressure = 0.0f;

// Monotonous changes in altitude
int monotonousChanges = 0;

// Rocket status
bool hasLiftedOff = false;
// END GLOBAL VARIABLES --------------------------------------------------------

// Function to setup the serial port, IMU and barometer
void setup()
{
  // Inicialization of serial port
  Serial.begin(9600);
  // Comment this loop when testing without computer
  while (!Serial)
    ;
  Serial.println("Started");

  // Uncomment this code if you want to use the IMU
  /*
  // Start IMU
  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  */

  // Start barometer
  if (!BARO.begin())
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1)
      ;
  }

  /* Calibrate barometer to get initial altitude
   */
  // Calibrate barometer with parameter received via serial port
  bool calibrated = false;

  // Loop until the barometer is calibrated
  /*while(!calibrated){
    Serial.println("Calibrating barometer...");
    String msg = Serial.readString();
    // String msg = "Initial _Altitude:X.XX";
    if (msg.startsWith("Initial _Altitude:"))
    {
      initialAltitude = msg.substring(18).toFloat();
      Serial.print("Initial altitude: ");
      Serial.println(initialAltitude);
      // Exit loop
      calibrated = true;
    }
    else
    {
      Serial.println("Bad command - Initial altitude not set");
    }
  }*/

  initialAltitude = 180.0f; // Remove
  initialPressure = BARO.readPressure();
}

// Function to check if the rocket is falling
bool isFalling()
{
  if (currentAltitude > lastAltitude) {
    // The rocket is ascending
    monotonousChanges = 0;
  } else if (currentAltitude < lastAltitude) {
    // The rocket is descending

    Serial.println("Se detecta cambio de monotonía");

    monotonousChanges++;
    
    Serial.print("Cambios de monotonía: ");
    Serial.println(monotonousChanges);

    if (monotonousChanges == 3 && hasLiftedOff) {
      /*At least 3 consecutive changes in monotonicity have occurred, the rocket
       *has reached its apogee and started descending.
       */
      Serial.println("Cohete descendiendo");
      return true;
    }
  }
  return false;
}

// Function to continue sending telemetry after deploying the parachute
void continueTelemetry()
{
  while(true){
    // TODO: send data of sensors to ground station or store in memory
  }
}

// Function to deploy the parachute
void checkDeployParachute()
{
  if (isFalling() && hasLiftedOff)
  {
    Serial.println("Desplegamos paracaídas!!");
    // TODO: Poner pin a HIGH para desplegar paracaídas

    continueTelemetry();
  }
  else
  {
    Serial.println("Subiendo o no hemos despegado");
  }
}

// Function to check the altitude and store it for checking if the rocket is falling
void checkAltitude()
{
  // Read pressure (kPa)
  float currentPressure = BARO.readPressure();
  Serial.print("Presión (kPa): ");
  Serial.println(currentPressure);

  // Update last altitude
  lastAltitude = currentAltitude;

  // Convert pressure to meters
  currentAltitude = 44330 * ( 1 - pow(currentPressure/initialPressure, 1/5.255) );
  Serial.print("Altitud relativa (m): ");
  Serial.println(currentAltitude);

  // Check if the rocket has lifted off
  if(!hasLiftedOff && (currentAltitude > minAltitudeToDeployParachute)) {
    hasLiftedOff = true;
  }
}

// Main function
void loop()
{
  checkAltitude();

  checkDeployParachute();

  // Delay to avoid overloading the serial port
  // TODO: minimize delay
  delay(300);
}
