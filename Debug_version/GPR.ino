//////////////////////////////////
//     <c> Mirel Paun 2024      //
//////////////////////////////////

//sampling period is 224 us
//sweep bandwidth = 539000000;        //GPR bandwidth [Hz]; 355.7 MHz - 894.7 MHz
//after each wheel trigger sends to PC 256 uint16 values (highbyte first)
//representing 256 time domain samples of 2 bytes each (512 bytes in total)

#define MCP4725_Address 96            //DAC adress
#define pin_ADC A15                   //ADC input pin
#define wheel_trigger 3
#include <Wire.h>

const word no_of_samples = 256;       //no. of time domain samples

//global variables
word i, j, samples[no_of_samples];
//modulator signal ramp that compensates JTOS-850VW+ nonlinearity
const word modulator_sig[256] = {111,119,127,134,142,150,158,166,174,182,190,198,206,214,222,230,239,248,256,265,274,282,291,299,308,317,325,334,343,351,360,369,377,386,394,403,412,420,429,438,447,456,466,475,485,494,504,513,523,532,542,551,561,570,580,589,599,608,618,627,637,646,656,665,675,685,695,705,715,725,735,745,755,765,775,785,795,805,815,825,835,845,855,865,875,885,895,906,917,928,939,950,960,971,982,993,1004,1015,1026,1036,1047,1058,1069,1080,1091,1102,1113,1125,1137,1149,1161,1172,1184,1196,1208,1220,1232,1244,1256,1268,1280,1292,1303,1315,1327,1340,1353,1366,1379,1392,1405,1418,1431,1444,1457,1470,1483,1496,1509,1522,1535,1548,1562,1576,1590,1605,1619,1633,1647,1661,1675,1689,1704,1718,1732,1746,1760,1775,1790,1806,1821,1837,1853,1868,1884,1899,1915,1931,1946,1962,1977,1993,2010,2027,2044,2061,2077,2094,2111,2128,2145,2162,2179,2196,2213,2232,2251,2270,2289,2308,2328,2347,2366,2385,2404,2423,2443,2462,2481,2500,2519,2538,2558,2577,2596,2615,2634,2654,2675,2697,2720,2742,2764,2786,2808,2830,2853,2875,2899,2923,2948,2972,2996,3021,3045,3069,3094,3120,3147,3174,3201,3228,3255,3282,3308,3337,3367,3396,3426,3456,3486,3516,3546,3578,3611,3643,3676,3708,3741,3774,3809,3844,3880,3915,3950,3985};

bool connected = false;
char command;

//------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(wheel_trigger, INPUT);
  delay(500);

  //bring DAC at start
  Wire.beginTransmission(MCP4725_Address);
  Wire.write( highByte(modulator_sig[0]) );     //MSB
  Wire.write( lowByte(modulator_sig[0]) );      //LSB
  Wire.endTransmission();
  delay(100);
  
  //wait for Start command from PC
  while (connected == false)
  {
    if (Serial.available() > 0)
    {
      command = Serial.read();
      if (command == 'S') connected = true;
    }
  }  
}

//------------------------------------------------------------------------
void loop()
{
  // Wait for trigger wheel signal
  while (digitalRead(wheel_trigger) == 0)
  {
    ;
  }
  delay(25);
  while (digitalRead(wheel_trigger) == 1)
  {
    ;
  }
  delay(25);
 
  // Generate modulation signal and sample radar echo
  for (i = 0; i < 256; i++)
    { 
      j = modulator_sig[i];
      // Write at DAC  
      Wire.beginTransmission(MCP4725_Address);
      Wire.write( highByte(j) );        //MSB
      Wire.write( lowByte(j) );         //LSB
      Wire.endTransmission();
      // Read from ADC
      samples[i]=((analogRead(pin_ADC)));
    }
  //Bring DAC at start
  Wire.beginTransmission(MCP4725_Address);
  Wire.write( highByte(modulator_sig[0]) );        //MSB
  Wire.write( lowByte(modulator_sig[0]) );         //LSB
  Wire.endTransmission();

  //Send one scan (256 time domain samples) over Serial connection
  for (i = 0; i < 256; i++)
  {
    Serial.write(highByte(samples[i]));
	  Serial.write(lowByte(samples[i]));
  }
}
