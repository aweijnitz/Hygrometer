#define  DRIVE1  6   // Digital pins used to drive square wave for the EFS-10
#define  DRIVE2  7
#define  HSENSE  A0  // analog input hygrometer (EFS-10)
#define  TSENSE  A1  // analog input temp sensor (LM335)

#define  V_DIVIDER 100000       // Using a 100k Ohm resistor as voltage divider
#define  TEMP_CALIBRATION -1.1f  // Factor to adjust the temperature sensor reading ("software calibration")
#define  LOG_INTERVAL  8192   // Throttles logging to terminal. 8192
#define  STEP_SIZE_TEMP 5.0f     // Temperature step size in lookup table
#define  STEP_SIZE_HUM 10.0f     // Humidity step size in lookup table

#define  PERIOD_US 1000  // for 1kHz, or use 10000 for 100Hz
#define  HALF_PERIOD_US (PERIOD_US/2)
#define  BAUD_RATE 115200

#define NR_COLS 8  // Lookup table dimensions
#define NR_ROWS 5  // ;;

#define  DEBUG  true  // Send debug info over serial.

/* 
   Hygrometer characteristics, impedance at different temps.
   From data-sheet at 
   http://www.produktinfo.conrad.com/datenblaetter/150000-174999/156545-da-01-en-Feuchtesensor_EFS10.pdf
*/
float lookupTable[NR_ROWS][NR_COLS] = {
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
  Serial.begin(BAUD_RATE);
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
  Work out impedance and finally realtive humidity, based on:
  - Current humidity sensor voltage reading
  - Current temp
  - Lookup table from data sheet stored in lookupTable[][]
  - Interpolation between neighbouring impedance values
  - Interpolation between neighbouring humidity values
  
  The circuit will read about 2.5V at (20C, 50% relative humidity)
*/
float voltToHumidity(float humidityVolt, float temp) {

  // The final result will end up in this variable.
  float relativeHumidity = 0.0f;

  // Calculate the impedance
  // impedance = U_measured/(current through R2). See circuit
  float imp = humidityVolt/((5.0 - humidityVolt)/V_DIVIDER); 

  // Calculate nearest temp rows and humidity columns (indices) that contain
  // the measured point (temp, imp) in the lookup table.
  //
  
  // The temperature rows
  byte tIndexL = (byte) (temp/5 - 3); // -3 is there to offset the index since the complete lookup table is not ported from the data sheet
  byte tIndexH = tIndexL + 1;
  
  // At this point we have an idea about which temperature rows to look in (tIndexL and tIndexH in lookupTable[][]).
  // Next: Find the two closest impedance cells based on r and  pick out the corresponding four
  // humidity values from the lookup table and then interpolate to approximate the humidity.
  //
  byte hIndexL = NR_COLS-1;
  byte hIndexH = 0;
  
  /*
  DEBUG: humidityVolt: 1.19 temp: 22.62 impedance 31183.95 
  tIndexL: 1 tIndexH: 2 
  hIndexL: 5 hIndexH: 4 
  rH_impLow_tL: 70.00 rH_impLow_tH: 60.00 rH_impHigh_tL: 70.00 rH_impHigh_tH: 60.00 
  tF: 0.52 hFL: 3.60 hFH: -1.18 
  rH0: 106.00 rH1: 48.18

106.00,22.62

  */
  
  while (lookupTable[tIndexL][hIndexL] < imp && hIndexL-- > 0);   
  /*
  if(hIndexL < NR_COLS-1) // The while loop will overshoot with one, so adjust index 
    hIndexL++;
    */
  while (lookupTable[tIndexH][hIndexH] > imp && hIndexH++ < NR_COLS);  
 /* 
  if(hIndexH > 0)  
    hIndexH--;
  */
  float tLow = (float)(tIndexL+3) * STEP_SIZE_TEMP; 
  float tHigh = (float)(tIndexH+3) * STEP_SIZE_TEMP;
  float rH_impLow_tL = 20.0f + hIndexL * STEP_SIZE_HUM; // a Table starts at rH 20%
  float rH_impLow_tH = 20.0f + hIndexH * STEP_SIZE_HUM; // b 
  float rH_impHigh_tL = 20.0f + hIndexL * STEP_SIZE_HUM; //c Table starts at rH 20%
  float rH_impHigh_tH = 20.0f + hIndexH * STEP_SIZE_HUM; //d
  
 /*
   Do interpolation in 2d to estimate relative humidity at (temp, imp) given the values in the table
  
    EXAMPLE: 
    humidityVolt: 1.25 temp: 22.62 impedance 33480.89 
    tIndexL: 1 tIndexH: 2 hIndexL: 4 hIndexH: 4 
    rH_impLow_tL: 60.00 rH_impLow_tH: 60.00 rH_impHigh_tL: 60.00 rH_impHigh_tH: 60.00 
    tF: 0.52 hFL: -0.12 hFH: -0.20 rH0: 58.84 rH1: 58.02
  
    relative humidity: 58.84%
  */
  float rH0, rH1, tF, hFL, hFH;
  if(hIndexL > 0 && hIndexH < NR_COLS && tIndexH < NR_ROWS-2) {
    tF = (temp - tLow)/STEP_SIZE_TEMP; // Temperature interpolation factor
    
    // Humidity sensor impedance steps are not equidistant for different temperatures, so we have to calculate
    // two interpolation factors for high and low value.
    hFL = (imp - lookupTable[tIndexL][hIndexL])/(lookupTable[tIndexL][hIndexL-1] - lookupTable[tIndexL][hIndexL]);
    hFH = (imp - lookupTable[tIndexL][hIndexH])/(lookupTable[tIndexH][hIndexH-1] - lookupTable[tIndexH][hIndexH]);
    
    rH0 = rH_impLow_tL + hFL*STEP_SIZE_HUM;
    rH1 = rH_impLow_tH + hFH*STEP_SIZE_HUM;
    
    relativeHumidity = rH0 + tF*((rH_impHigh_tL - rH_impLow_tL)/(rH1 - rH0));
  }
  else // Edge case.
    relativeHumidity = 0.0f; // TODO: linear iterpolation here

  if(DEBUG && printMe%LOG_INTERVAL == 0) {
    Serial.print("DEBUG: humidityVolt: ");
    Serial.print(humidityVolt);
    Serial.print(" temp: ");
    Serial.print(temp);
    Serial.print(" impedance ");
    Serial.print(imp);

    Serial.print(" tIndexL: ");
    Serial.print(tIndexL);
    Serial.print(" tIndexH: ");
    Serial.print(tIndexH);
    Serial.print(" hIndexL: ");
    Serial.print(hIndexL);
    Serial.print(" hIndexH: ");
    Serial.print(hIndexH);

    Serial.print(" rH_impLow_tL: ");
    Serial.print(rH_impLow_tL);
    Serial.print(" rH_impLow_tH: ");
    Serial.print(rH_impLow_tH);
    Serial.print(" rH_impHigh_tL: ");
    Serial.print(rH_impHigh_tL);
    Serial.print(" rH_impHigh_tH: ");
    Serial.print(rH_impHigh_tH);
    
    Serial.print(" tF: ");
    Serial.print(tF);
    Serial.print(" hFL: ");
    Serial.print(hFL);
    Serial.print(" hFH: ");
    Serial.print(hFH);
    
    Serial.print(" rH0: ");
    Serial.print( rH0);
    Serial.print(" rH1: ");
    Serial.println( rH1);
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



