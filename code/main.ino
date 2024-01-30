#include <Arduino_BMI270_BMM150.h>  //  Arduino BMI270 - Simple Gyroscope.
                                    //  Arduino BMI270 - Simple Accelerometer
                                    //  Arduino BMM150 - Simple Magnetometer
#include <Arduino_LPS22HB.h>        //  LPS22HB - Read Pressure and Barometer temperature.
#include <Arduino_HS300x.h>         //  HS300x - Read Sensors (Temp. Humidity).

#define minAltitudeToDeployParachute 2 // 20 meters minimum to deploy parachute
#define parachutePin 5
//#define alertPin 5

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

// TODO: comprobar que el Serial1 está disponible es necesario?

// Function to setup the Serial1 port, IMU and barometer
void setup()
{
  pinMode(parachutePin, OUTPUT);
  //pinMode(alertPin, OUTPUT);
  digitalWrite(parachutePin, LOW);
  //digitalWrite(alertPin, LOW);
  // Configurate pins 4 and 5 as output

  warningF = false;
  errorF = false;
  
  // Inicialization of Serial1 port
  Serial1.begin(9600);
  // Comment this loop when testing without computer
  while (!Serial1)
    ;
  Serial1.println("#INFO: Serial1 Port Started.");

  // ############# Initialize sensors ###########
  if (!BARO.begin()) {
    Serial1.println("#ERROR: Failed to initialize pressure sensor!");
    while (1)
      ;
  }

  if (!IMU.begin()) {
    Serial1.println("#ERROR: Failed to initialize IMU!");
    errorF = true;
    while (1)
      ;
  }

  if (!HS300x.begin()) {
    Serial1.println("#WARNING: Failed to initialize humidity temperature sensor!");
    warningF = true;
    while (1)
      ;
  }

  Serial1.print("#INFO: Gyroscope sample rate = ");
  Serial1.print(IMU.gyroscopeSampleRate());
  Serial1.println(" Hz");
  Serial1.println();
  Serial1.println("#INFO: Gyroscope in degrees/second");
  Serial1.println("X\tY\tZ");

  Serial1.print("#INFO: Accelerometer sample rate = ");
  Serial1.print(IMU.accelerationSampleRate());
  Serial1.println(" Hz");
  Serial1.println();
  Serial1.println("#INFO: Acceleration in G's");
  Serial1.println("X\tY\tZ");

  Serial1.print("#INFO: Magnetic field sample rate = ");
  Serial1.print(IMU.magneticFieldSampleRate());
  Serial1.println(" Hz");
  Serial1.println();
  Serial1.println("#INFO: Magnetic Field in uT");
  Serial1.println("X\tY\tZ");

  // ##########################################

  /* Calibrate barometer to get initial altitude
   */
  // Calibrate barometer with parameter received via Serial1 port
  bool calibrated = false;

  // Loop until the barometer is calibrated ###
  while(!calibrated){
    Serial1.println("#INFO: Calibrating barometer...");
    String msg = Serial1.readString();
    // String msg = "Initial_Altitude:X.XX";
    if (msg.startsWith("Initial_Altitude:"))
    {
      initialAltitude = msg.substring(17).toFloat();
      Serial1.print("Initial altitude: ");
      Serial1.println(initialAltitude);
      // Exit loop
      calibrated = true;
    }
    else
    {
      Serial1.println("Bad command - Initial altitude not set");
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

    Serial1.println("Se detecta cambio de monotonía");

    monotonousChanges++;
    
    Serial1.print("Cambios de monotonía: ");
    Serial1.println(monotonousChanges);

    if (monotonousChanges == 3 && hasLiftedOff) {
      /*At least 3 consecutive changes in monotonicity have occurred, the rocket
       *has reached its apogee and started descending.
       */
      Serial1.println("Cohete descendiendo");
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
  //Serial1.print('$,');
  Serial1.print(pressure);
  Serial1.print(',');
  Serial1.print(temperatureBARO);
  Serial1.print(',');
  currentAltitude = 44330 * ( 1 - pow(pressure/initialPressure, 1/5.255) );
  temperature = HS300x.readTemperature();
  humidity = HS300x.readHumidity();
  Serial1.print(currentAltitude);
  Serial1.print(',');
  Serial1.print(temperature);
  Serial1.print(',');
  Serial1.print(humidity);
  Serial1.print(',');


  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Gx, Gy, Gz);

    Serial1.print(Gx);
    Serial1.print(',');
    Serial1.print(Gy);
    Serial1.print(',');
    Serial1.print(Gz);
    Serial1.print(',');
  } else {
    Serial1.println("#ERROR: Gyroscope not available!");
    errorF = true;
  }

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Ax, Ay, Az);

    Serial1.print(Ax);
    Serial1.print(',');
    Serial1.print(Ay);
    Serial1.print(',');
    Serial1.print(Az);
    Serial1.print(',');
  } else {
    Serial1.println("#ERROR: Accelerometer not available!");
    errorF = true;
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(Mx, My, Mz);
  } else {
    Serial1.println("#ERROR: Magnetometer not available!");
    errorF = true;
  }
    
  Serial1.print(Mx);
  Serial1.print(',');
  Serial1.print(My);
  Serial1.print(',');
  Serial1.print(Mz);
  //Serial1.println(',;');
  Serial1.println();
}

// Function to deploy the parachute
void checkDeployParachute()
{
  if (isFalling() && hasLiftedOff)
  {
    Serial1.println("Desplegamos paracaidas!!");
    // Activate alert sound 
    //digitalWrite(alertPin, HIGH);
    //delay(3000);
    // Deactivate alert sound 
    //digitalWrite(alertPin, LOW);
    
    // Deploy parachute
    digitalWrite(parachutePin, HIGH);

    while(true){
      checkTelemetry();
      delay(200);
    }
    
  }
  else
  {
    Serial1.println("Subiendo o no hemos despegado");
  }
}

// Function to check the altitude and store it for checking if the rocket is falling
void checkAltitude()
{
  // Read pressure (kPa)
  float currentPressure = BARO.readPressure();
  Serial1.print("Presión (kPa): ");
  Serial1.println(currentPressure);

  // Update last altitude
  lastAltitude = currentAltitude;

  // Convert pressure to meters
  currentAltitude = 44330 * ( 1 - pow(currentPressure/initialPressure, 1/5.255) );
  Serial1.print("Altitud relativa (m): ");
  Serial1.println(currentAltitude);

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
      if (Ay > 2.0f || Ay < -2.0f){
        startTime = millis();
        Serial1.println("#INFO: Start timer!");
      }
    } else {
      Serial1.println("#ERROR: Accelerometer not available!");
      errorF = true;
    }
  }

  currentTime = millis();
  if ((currentTime - startTime) > timerLimit && startTime != 0){
    Serial1.println("Desplegamos paracaídas por timer!!");
    // Activate alert sound 
    //digitalWrite(alertPin, HIGH);
    //delay(3000);
    // Deactivate alert sound
    //digitalWrite(alertPin, LOW);
    
    // Deploy parachute
    digitalWrite(parachutePin, HIGH);

    while(true){
      checkTelemetry();
      delay(100);
    }
  }

  // Delay to avoid overloading the Serial1 port
  // TODO: minimize delay
  delay(200);
}
