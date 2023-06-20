#include "Arduino_BMI270_BMM150.h"
#include "Arduino_LPS22HB.h"

float x, y, z;
int degreesX = 0;
int degreesY = 0;

// Acceleration array
float accArray[] = {0, 0, 0, 0, 0};
int idx = 0;

int hasLiftedOff = 0;
float initialPressure, currentPressure;

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;
  Serial.println("Started");

  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }
  
  if (!BARO.begin()) {

    Serial.println("Failed to initialize pressure sensor!");

    while (1);

  }

  initialPressure = BARO.readPressure(MILLIBAR);
  

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
}

int isFalling() {
  for (int i = 0; i < 5; i++) {
    if (accArray[i] > -0.3) {
      return 0;
    }
  }
  return 1;
}

void loop()
{
  float x, y, z;

  if (IMU.accelerationAvailable())
  {
    IMU.readAcceleration(x, y, z);

    /*Serial.print("x: ");
    Serial.print(x);
    Serial.print(" y: ");
    Serial.print(y);
    Serial.print(" z: ");
    Serial.print(z);
    Serial.println();*/
    //Serial.println(z);
    accArray[idx] = z;
    if (idx == 5) {
      idx = 0;
    } else {
      idx++;
    }
    if (isFalling()) {
      //Serial.println("Cayendo!!");
    }else {
      //Serial.println("Subiendo!!");
    }
  }

  currentPressure = BARO.readPressure(MILLIBAR);
  if (!hasLiftedOff && (initialPressure - currentPressure) > 1) {
    hasLiftedOff = 1;
  }

  if (hasLiftedOff) {
    Serial.println("Estamos volando!!");
    } else {
      Serial.println("No hemos despegado aun");}


  // print an empty line

  Serial.println();
  delay(1000);
}
