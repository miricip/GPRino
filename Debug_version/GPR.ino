//////////////////////////////////
//     <c> Mirel Paun 2024      //
//////////////////////////////////

//sampling period is 224.38 us
//sweep bandwidth = 587000000;        //GPR bandwidth [Hz]; 910 MHz - 323 MHz
//after each wheel trigger sends to PC 256 uint16 values (highbyte first)
//representing 256 time domain samples of 2 bytes each (512 bytes in total)

#define MCP4725_Address 96            //DAC adress
#define pin_ADC A15                   //ADC input pin
#define wheel_trigger 3
#include <Wire.h>

const word no_of_samples = 256;       //no. of time domain samples

//global variables
word i, samples[no_of_samples];
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

  //bring DAC at 0
  Wire.beginTransmission(MCP4725_Address);
  Wire.write( 0 );        //MSB
  Wire.write( 0 );        //LSB
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
  for (i = 0; i <= 4080; i = i + 16) // DAC goes from 0 - 4.98V
  {
    // Write to DAC
    Wire.beginTransmission(MCP4725_Address);
    Wire.write( highByte(i) );        //MSB
    Wire.write( lowByte(i) );         //LSB
    Wire.endTransmission();
    // Read from ADC
    samples[i >> 4] = analogRead(pin_ADC); // >>4 means /16
  }
  //Bring DAC to 0
  Wire.beginTransmission(MCP4725_Address);
  Wire.write( 0 );        //MSB
  Wire.write( 0 );        //LSB
  Wire.endTransmission();

  //Send one scan (256 time domain samples) over Serial connection
  for (i = 0; i < 256; i++)
  {
    Serial.write(highByte(samples[i]));
	  Serial.write(lowByte(samples[i]));
  }
}
