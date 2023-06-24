#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"

#define pressureThreshold 2.5 // 2.5 millibar equals 20 meters of air aprox

// Acceleration measurements
float x, y, z;
float accArray[] = {0, 0, 0, 0, 0};
int idx = 0;

// Pressure measurements
float initialPressure, currentPressure;

// Rocket status
bool hasLiftedOff = false;

void setup()
{
  Serial.begin(9600);
  // Comment this loop when testing without computer
  while (!Serial)
    ;
  Serial.println("Started");

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

  // Start BARO
  if (!BARO.begin())
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1)
      ;
  }

  // Read initial pressure
  initialPressure = BARO.readPressure(MILLIBAR);
}

int isFalling()
{
  // Check if all values are negative
  /*for (int i = 0; i < 5; i++)
  {
    if (accArray[i] > -0.3)
    {
      return 0;
    }
  }
  return 1;*/

  // Average acceleration
  int sum = 0;
  for (int i = 0; i < 5; i++)
  {
    sum += accArray[i];
  }
  // Check 0.2 value
  // Check if average acceleration is less than -0.2g
  if (sum / 5 < -0.2)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void loop()
{
  if (IMU.accelerationAvailable())
  {
    // Read acceleration in 3 axis
    IMU.readAcceleration(x, y, z);
    Serial.print("Aceleración Y:");
    Serial.println(y);

    // Store acceleration in array
    accArray[idx] = y;

    // Increment index
    idx++;
    if (idx == 5)
    {
      idx = 0;
    }

    if (isFalling() && hasLiftedOff)
    {
      Serial.println("Desplegamos paracaídas!!");
    }
    else
    {
      Serial.println("Subiendo o no hemos despegado");
    }
  }

  currentPressure = BARO.readPressure(MILLIBAR);
  Serial.print("Presión: ");
  Serial.println(currentPressure);

  // If we haven't already lifted off and there's a change in pressure greater than pressureThreshold
  if (!hasLiftedOff && (initialPressure - currentPressure) > pressureThreshold)
  {
    // We've lifted off!
    hasLiftedOff = true;
  }
  delay(300);
}
