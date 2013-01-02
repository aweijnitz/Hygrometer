#define  DRIVE1  6   // Digital pins used to drive square wave for the EFS-10
#define  DRIVE2  7
#define  HSENSE  A0  // analog input hygrometer (EFS-10)
#define  TSENSE  A1  // analog input temp sensor (LM335)

#define  V_DIVIDER 100000       // Using a 100k Ohm resistor as voltage divider
#define  TEMP_CALIBRATION -1.0  // Factor to adjust the temperature sensor reading ("software calibration")
#define  LOG_INTERVAL  8192   // Throttles logging to terminal. 8192

#define  PERIOD_US 1000  // for 1kHz, or use 10000 for 100Hz
#define  HALF_PERIOD_US (PERIOD_US/2)

#define  DEBUG  true  // Send debug info over serial.

/* 
  Hygrometer characteristics, impedance at different temps.
  From data-sheet at 
  http://www.produktinfo.conrad.com/datenblaetter/150000-174999/156545-da-01-en-Feuchtesensor_EFS10.pdf
*/
float lookupTable[5][8] = {
/* T/H      20%            30%            40%           50%           60%             70%             80%            90%           */
/* 15C */  {6364*1000.0f,  1803*1000.0f,  543*1000.0f,  166*1000.0f,  55.64*1000.0f,  20.94*1000.0f,  8.07*1000.0f,  3.26*1000.0f},
/* 20C */  {4500*1000.0f,  1300*1000.0f,  398*1000.0f,  125*1000.0f,  43.0*1000.0f,   17.0*1000.0f,   6.85*1000.0f,  2.85*1000.0f},
/* 25C */  {2890*1000.0f,   900*1000.0f,  270*1000.0f,  81*1000.0f,   33.0*1000.0f,   13.0*1000.0f,   5.30*1000.0f,  2.20*1000.0f},
/* 30C */  {2100*1000.0f,   670*1000.0f,  210*1000.0f,  66*1000.0f,   25.50*1000.0f,  10.20*1000.0f,  4.28*1000.0f,  1.85*1000.0f},
/* 35C */  {1652*1000.0f,   530*1000.0f,  168*1000.0f,  54*1000.0f,   21.54*1000.0f,   8.69*1000.0f,  3.71*1000.0f,  1.62*1000.0f}
  };
  
// State variables
//
unsigned long prev_time;  // Used to generate square wave to drive humidity sensor
byte phase = 0;           // Used to generate square wave to drive humidity sensor
float prevVoltage = 0.0f; // Used for keping a moving average of humidity sensor readings
int printMe = 0;          // just a counter to trottle logging of values 

void setup() {
  Serial.begin(115200);
  pinMode (DRIVE1, OUTPUT);
  pinMode (DRIVE2, OUTPUT);
  prev_time = micros ();
}

void loop() {

  // Generate 1kHz square wave to drive sensor. Busy wait, so blocks here.
  squareWaveSensorDrive();
  
  float voltage = readHumiditySensor();  
  float temp = readTemp();
  float realtiveHumidity = voltToHumidity(voltage, temp);
  
  if(printMe++%LOG_INTERVAL == 0) {
    outputResult(realtiveHumidity, temp); 
  } 
}


/*
  Work out impedance and finally realtuve humidity, based on:
  - Current humidity sensor voltage reading
  - Current temp
  - Lookup table from data sheet stored in lookupTable[][]
  - Interpolation between neighbouring impedance values
  
  The circuit will read about 2.5V at (20C, 50% relative humidity)
*/
float voltToHumidity(float humidityVolt, float temp) {
  float relativeHumidity = 0.0f;
  float i = (5.0 - humidityVolt)/V_DIVIDER; // Calculate current through the sensor 
  float r = humidityVolt/i;   // Calculate impedance, using the current
  
  // Figure out best match
  byte tmp = (byte) rint(temp);
  byte t = (tmp - (tmp % 5))/5;
  byte tIndex = t - 3; // -3 is there to offset the index since the complete lookup table is not ported from the data sheet

  // At this point we have an idea about which temperature row to look in (tIndex in lookupTable[][]).
  // Next: Find the two closest impedance cells based on r and  pick out the corresponding 
  // humidity values from the other lookup table and then interpolate between the two
  // to approximate the humidity.
  //
  byte rIndex = 0;
  do { } while (lookupTable[tIndex][rIndex] > r && rIndex++ < 8); 
  
  if(rIndex > 0) 
    rIndex--;
  
  float rHLow = 20.0f + rIndex*10; // Table starts at rH 20%
  if(rIndex > 0 && rIndex < 8) {
    relativeHumidity = rHLow + 10*((r-lookupTable[tIndex][rIndex])/(lookupTable[tIndex][rIndex-1] - lookupTable[tIndex][rIndex]));
    
    // Read from row next temp over in table to adjust reading
/*    float rTemp = 0.0;
    if(rIndex < 7 && (temp - ()) < 3)
      rTemp = rHLow + 10*((r-lookupTable[tIndex][rIndex])/(lookupTable[tIndex][rIndex-1] - lookupTable[tIndex][rIndex]));    
 
    if(rTemp > 0) { // Do extra interpolation to adjust for temp reading bewteen values (ex. 23.7 is between 20 and 25C rows)
    }
 */
  }
  else // TODO: Edge case, I believe there is a bug here. Need to check.
    relativeHumidity = rHLow + 10*((r-lookupTable[tIndex][rIndex])/(lookupTable[tIndex][rIndex]));

  if(DEBUG && printMe%LOG_INTERVAL == 0) {
    Serial.print("DEBUG: humidityVolt: ");
    Serial.print(humidityVolt);
    Serial.print(" temp: ");
    Serial.print(temp);
    Serial.print(" rint(temp): ");
    Serial.print(rint(temp));
    Serial.print(" r: ");
    Serial.print(r);
    Serial.print(" rIndex: ");
    Serial.print(rIndex);
    Serial.print(" tIndex: ");
    Serial.print(tIndex);
    Serial.print(" rHLow: ");
    Serial.print( rHLow);
    Serial.print(" interpolation: ");
    Serial.println(10*((r-lookupTable[tIndex][rIndex])/(lookupTable[tIndex][rIndex-1] - lookupTable[tIndex][rIndex])));    
  }
  return relativeHumidity;
}


/*
  Read sensor, convert to volts and apply a basic moving average to minimize jitter.
  Returns the reading in Volts.
*/
float readHumiditySensor() {
  int val = analogRead (HSENSE);
  val = analogRead (HSENSE); // second read allows much longer for high impedance reading to settle
  if (phase == 0)
    val = 1023 - val;  // reverse sense on alternate half cycles
  float v = 5.0 * val / 1024; // Convert to volts
  float voltage = (v + prevVoltage) / 2; // Moving average
  prevVoltage = voltage;
  phase = 1 - phase;
  
  return voltage;
}


/*
  Read sensor, convert to volts and then to Celsius.
  Returns current reading in Celsius.
*/
float readTemp() {
  float val = analogRead (TSENSE);
  val = analogRead (TSENSE);
  
  // Convert reading to Celsius
  float v = val*5.0/1024.0f; // Now in Volts
  v *= 100; // Now in degrees Kelvin
  v += TEMP_CALIBRATION;
  return v - 273.15; // Return in Celsius
}


/* 
  Generates a square wave to drive the humidity sensor..
  Blocks for half the period.
*/
void squareWaveSensorDrive() {
  while (micros () - prev_time < HALF_PERIOD_US) {}  //wait for next 0.5ms time segment
  prev_time += HALF_PERIOD_US;   // set up for the next loop
  digitalWrite (DRIVE1, phase == 0);   // antiphase drive
  digitalWrite (DRIVE2, phase != 0);
}


/*
  Output read values at regular intervals.
  Prints to Serial.
*/
void outputResult(float realtiveHumidity, float temp) {
  Serial.print(realtiveHumidity);
  Serial.print(",");
  Serial.println(temp);
}


/*
  Round float to nearest integer. Return as float.
*/
float rint(float f) {
  float result = 0;
  float fraction = f - ((byte) f);
  
  if(fraction < 0.5)
    result = f - fraction;
  else
    result = f + 1 - fraction ;
  
  return result;
}



