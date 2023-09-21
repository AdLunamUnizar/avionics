#include <Arduino_BMI270_BMM150.h>  //  Arduino BMI270 - Simple Gyroscope.
                                    //  Arduino BMI270 - Simple Accelerometer
                                    //  Arduino BMM150 - Simple Magnetometer
#include <Arduino_LPS22HB.h>        //  LPS22HB - Read Pressure and Barometer temperature.
#include <Arduino_HS300x.h>         //  HS300x - Read Sensors (Temp. Humidity).

#define minAltitudeToDeployParachute 20 // 20 meters minimum to deploy parachute
#define parachutePin 4
#define alertPin 5

// GLOBAL VARIABLES ------------------------------------------------------------
int addr = 0; // Memory address where the data start to be stored

// Acceleration measurements
float Ax = 0.0f, Ay = 0.0f, Az = 0.0f;

bool warningF;
bool errorF;

// Pressure measurements
float initialAltitude = 0.0f, lastAltitude = 0.0f, currentAltitude = 0.0f;
float initialPressure = 0.0f;

// Monotonous changes in altitude
int monotonousChanges = 0;

// Rocket status
bool hasLiftedOff = false;

// Time
unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned long timerLimit = 11000; // 11 seconds limit to deploy parachute
// END GLOBAL VARIABLES --------------------------------------------------------

// TODO: comprobar que el serial está disponible es necesario?

// Function to setup the serial port, IMU and barometer
void setup()
{  
  digitalWrite(parachutePin, LOW);
  digitalWrite(alertPin, LOW);
  // Configurate pins 4 and 5 as output
  pinMode(parachutePin, OUTPUT);
  pinMode(alertPin, OUTPUT);

  warningF = false;
  errorF = false;
  
  // Inicialization of serial port
  Serial.begin(9600);
  // Comment this loop when testing without computer
  while (!Serial)
    ;
  Serial.println("#INFO: Serial Port Started.");

  // ############# Initialize sensors ###########
  if (!BARO.begin()) {
    Serial.println("#ERROR: Failed to initialize pressure sensor!");
    while (1)
      ;
  }

  if (!IMU.begin()) {
    Serial.println("#ERROR: Failed to initialize IMU!");
    errorF = true;
    while (1)
      ;
  }

  if (!HS300x.begin()) {
    Serial.println("#WARNING: Failed to initialize humidity temperature sensor!");
    warningF = true;
    while (1)
      ;
  }

  Serial.print("#INFO: Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("#INFO: Gyroscope in degrees/second");
  Serial.println("X\tY\tZ");

  Serial.print("#INFO: Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("#INFO: Acceleration in G's");
  Serial.println("X\tY\tZ");

  Serial.print("#INFO: Magnetic field sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("#INFO: Magnetic Field in uT");
  Serial.println("X\tY\tZ");

  // ##########################################

  /* Calibrate barometer to get initial altitude
   */
  // Calibrate barometer with parameter received via serial port
  bool calibrated = false;

  // Loop until the barometer is calibrated ###
  while(!calibrated){
    Serial.println("#INFO: Calibrating barometer...");
    String msg = Serial.readString();
    // String msg = "Initial_Altitude:X.XX";
    if (msg.startsWith("Initial_Altitude:"))
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
  }

  //initialAltitude = 180.0f; // Remove
  initialPressure = BARO.readPressure();
  // ##########################################
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

// Function to check and send telemetry
void checkTelemetry()
{
  // Telemetry variables
  float pressure;
  float temperatureBARO;
  float Gx, Gy, Gz;
  float Mx, My, Mz;
  float temperature;
  float humidity;
  
  pressure = BARO.readPressure();
  temperatureBARO = BARO.readTemperature();
  //Serial.print('$,');
  Serial.print(pressure);
  Serial.print(',');
  Serial.print(temperatureBARO);
  Serial.print(',');
  currentAltitude = 44330 * ( 1 - pow(pressure/initialPressure, 1/5.255) );
  temperature = HS300x.readTemperature();
  humidity = HS300x.readHumidity();
  Serial.print(currentAltitude);
  Serial.print(',');
  Serial.print(temperature);
  Serial.print(',');
  Serial.print(humidity);
  Serial.print(',');


  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gx, Gy, Gz);

    Serial.print(Gx);
    Serial.print(',');
    Serial.print(Gy);
    Serial.print(',');
    Serial.print(Gz);
    Serial.print(',');
  } else {
    Serial.println("#ERROR: Gyroscope not available!");
    errorF = true;
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);

    Serial.print(Ax);
    Serial.print(',');
    Serial.print(Ay);
    Serial.print(',');
    Serial.print(Az);
    Serial.print(',');
  } else {
    Serial.println("#ERROR: Accelerometer not available!");
    errorF = true;
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(Mx, My, Mz);
  } else {
    Serial.println("#ERROR: Magnetometer not available!");
    errorF = true;
  }
    
  Serial.print(Mx);
  Serial.print(',');
  Serial.print(My);
  Serial.print(',');
  Serial.print(Mz);
  //Serial.println(',;');
  Serial.println();
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

    while(true){
      checkTelemetry();
      delay(200);
    }
    
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

  checkTelemetry();

  // Wait until acceleration is > 50m/s^2 aprox
  if(startTime == 0){
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(Ax, Ay, Az);
      if (Ay > 5.0f){
        startTime = millis();
      }
    } else {
      Serial.println("#ERROR: Accelerometer not available!");
      errorF = true;
    }
  }

  currentTime = millis();
  if ((currentTime - startTime) > timerLimit && startTime != 0){
    Serial.println("Desplegamos paracaídas por timer!!");
    // Activate alert sound 
    digitalWrite(alertPin, HIGH);
    delay(3000);
    // Deactivate alert sound 
    digitalWrite(alertPin, LOW);
    
    // Deploy parachute
    digitalWrite(parachutePin, HIGH);

    while(true){
      checkTelemetry();
      delay(200);
    }
  }

  // Delay to avoid overloading the serial port
  // TODO: minimize delay
  delay(200);
}
