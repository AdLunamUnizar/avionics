#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"
#include <SoftwareSerial.h>

#define minAltitudeToDeployParachute 20 // 20 meters minimum to deploy parachute
#define parachutePin 4
#define alertPin 5

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

// Serial LORA port
SoftwareSerial mySerial(2, 3);

// TODO: comprobar que el serial está disponible es necesario?

// Function to setup the serial port, IMU and barometer
void setup()
{
  digitalWrite(parachutePin, LOW);
  digitalWrite(alertPin, LOW);
  // Configurate pins 4 and 5 as output
  pinMode(parachutePin, OUTPUT);
  pinMode(alertPin, OUTPUT);
  
  // Inicialization of serial port
  Serial.begin(9600);
  // Comment this loop when testing without computer
  while (!Serial)
    ;
  Serial.println("Started");

  // Inicialization of LORA serial port
  mySerial.begin(9600);
  // Comment this loop when testing without computer
  while (!mySerial.available())
    ;
  mySerial.println("Started LORA serial port");

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
    mySerial.println("Failed to initialize pressure sensor!");
    while (1)
      ;
  }

  /* Calibrate barometer to get initial altitude
   */
  // Calibrate barometer with parameter received via serial port
  bool calibrated = false;

  // Loop until the barometer is calibrated
  while(!calibrated){
    Serial.println("Calibrating barometer...");
    mySerial.println("Calibrating barometer...");
    String msg = mySerial.readString();
    // String msg = "Initial_Altitude:X.XX";
    if (msg.startsWith("Initial_Altitude:"))
    {
      initialAltitude = msg.substring(18).toFloat();
      Serial.print("Initial altitude: ");
      mySerial.print("Initial altitude: ");
      Serial.println(initialAltitude);
      mySerial.println(initialAltitude);
      // Exit loop
      calibrated = true;
    }
    else
    {
      Serial.println("Bad command - Initial altitude not set");
      mySerial.println("Bad command - Initial altitude not set");
    }
  }

  //initialAltitude = 180.0f; // Remove
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
    mySerial.println("Se detecta cambio de monotonía");

    monotonousChanges++;
    
    Serial.print("Cambios de monotonía: ");
    Serial.println(monotonousChanges);
    mySerial.print("Cambios de monotonía: ");
    mySerial.println(monotonousChanges);

    if (monotonousChanges == 3 && hasLiftedOff) {
      /*At least 3 consecutive changes in monotonicity have occurred, the rocket
       *has reached its apogee and started descending.
       */
      Serial.println("Cohete descendiendo");
      mySerial.println("Cohete descendiendo");
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
    // Activate alert sound 
    digitalWrite(alertPin, HIGH);
    delay(3000);
    // Deactivate alert sound 
    digitalWrite(alertPin, LOW);
    
    // Deploy parachute
    digitalWrite(parachutePin, HIGH);

    continueTelemetry();
  }
  else
  {
    Serial.println("Subiendo o no hemos despegado");
    mySerial.println("Subiendo o no hemos despegado");
  }
}

// Function to check the altitude and store it for checking if the rocket is falling
void checkAltitude()
{
  // Read pressure (kPa)
  float currentPressure = BARO.readPressure();
  Serial.print("Presión (kPa): ");
  Serial.println(currentPressure);
  mySerial.print("Presión (kPa): ");
  mySerial.println(currentPressure);

  // Update last altitude
  lastAltitude = currentAltitude;

  // Convert pressure to meters
  currentAltitude = 44330 * ( 1 - pow(currentPressure/initialPressure, 1/5.255) );
  Serial.print("Altitud relativa (m): ");
  Serial.println(currentAltitude);
  mySerial.print("Altitud relativa (m): ");
  mySerial.println(currentAltitude);

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
