//////////////////////////////////
//     <c> Mirel Paun 2020      //
//////////////////////////////////

//sampling period is 224.38 us
//stored file format - the first number (2 bytes) contains the speed index, the following contain the time domain samples
#define Adresa_MCP4725 96             //DAC adress
#define pin_ADC A15                   //ADC input pin
#define pin_BAT A13                   //Battery voltage input pin
#define WHITE     0xFFFF              //color
#define BLACK     0x0000              //color
#define RED       0xF800              //color
#define GREEN     0x07E0              //color
#define DKBLUE    0x000D              //color
#define YELLOW    0xFFE0              // Yellow color
#define button_up 7
#define button_OK 6
#define button_down 5
#define step_sensor 3
#define SDC_CS 53                     //SD card control pin
#include <Wire.h>
#include <arduinoFFT.h>
#include <TFT_HX8357.h>
#include "Free_Fonts.h"
#include <SD.h>

TFT_HX8357 tft = TFT_HX8357();
arduinoFFT FFT = arduinoFFT();

File file;

const word battery_level[3] = {384, 392, 401};   //battery voltage threshold levels: low - med1 - med2 (V_bat/5V*1023*0.4)

//GPR global parameters
const double pas_deplasare_GPR = 0.12;    //GPR horizontal step [m]
const double banda = 587000000;           //GPR bandwidth [Hz]; 910 MHz - 323 MHz
const double amplitudine_maxima = 12000;  //maximum FFT magnitude value (for scaling)
const word nr_esant = 256;                //sample no.

//menus
const char *medii_propagare[] = {"Air - 0.3 m/ns", "Ice - 0.16 m/ns", "Dry sand - 0.15 m/ns", "Dry soil (Granite) - 0.13 m/ns", "Limestone - 0.12 m/ns", "Asphalt - 0.11 m/ns", "Concrete - 0.1 m/ns", "Moist soil - 0.09 m/ns", "Wet soil (Silt) - 0.07 m/ns", "Saturated sand (Clay) - 0.06 m/ns", "Sweet water - 0.03 m/ns", "Salt water - 0.01 m/ns"};
const double viteze[] = {300000000, 160000000, 150000000, 130000000, 120000000, 110000000, 100000000, 90000000, 70000000, 60000000, 30000000, 10000000};
const word adancimi[] = {16, 32, 64, 128}; //resolution multiples
const double pasi_adanc[] = 		{10, 5, 2.5, 1, 0.5, 0.25, 0.1, 0.05, 0.025, 0.01};
const word zecimale_pasi_adanc[] =  {0,  0, 1  , 0,   1,    2,   1,    2,     3,    2};
const word distante[] = {6, 12, 24, 48};

//graph constants
const word orig_x = 60, orig_y = 290, latime_grafic = 400, inaltime_grafic = 256; //graph origin coord. and dimensions [pixels]
//variabile globale afisare grafic
double max_dist, pas_dist, max_adanc, min_adanc, pas_adanc, rezolutie; // [m]
double c; //[m/s]
word nr_cel_rez_oriz, nr_cel_rez_vert, inaltime_cel_rez, latime_cel_rez, nr_zecimale_pasi_adanc, xpos, ypos, pas = 0;

//antenna coupling correction (anechoic chamber acquisition)
//const word corectie[nr_esant] = {497, 497, 477, 383, 251, 163, 125, 113, 146, 210, 305, 430, 550, 682, 801, 893, 947, 964, 922, 787, 654, 569, 521, 486, 455, 446, 451, 454, 439, 409, 377, 352, 337, 332, 323, 334, 342, 354, 371, 384, 397, 410, 420, 433, 449, 468, 496, 528, 560, 596, 637, 674, 705, 726, 733, 735, 735, 738, 749, 757, 760, 754, 731, 699, 657, 597, 520, 432, 342, 264, 213, 180, 164, 164, 173, 194, 222, 252, 288, 316, 350, 390, 425, 459, 491, 522, 548, 571, 590, 606, 624, 642, 660, 681, 694, 703, 706, 701, 692, 676, 651, 623, 590, 557, 528, 501, 477, 457, 443, 433, 429, 429, 431, 433, 439, 449, 462, 476, 492, 508, 525, 543, 566, 587, 604, 609, 603, 589, 570, 547, 519, 482, 434, 376, 326, 277, 233, 194, 159, 147, 167, 224, 306, 383, 449, 503, 545, 576, 601, 611, 615, 616, 617, 617, 616, 614, 613, 609, 602, 593, 584, 577, 571, 566, 559, 553, 545, 539, 533, 528, 524, 521, 518, 515, 510, 505, 500, 496, 493, 490, 485, 480, 477, 475, 474, 475, 476, 479, 484, 490, 496, 502, 508, 514, 522, 532, 538, 542, 541, 540, 538, 536, 536, 534, 531, 525, 520, 511, 503, 497, 491, 487, 483, 479, 473, 467, 468, 468, 466, 466, 466, 466, 467, 467, 470, 468, 467, 467, 466, 466, 465, 465, 467, 468, 467, 468, 467, 471, 473, 475, 477, 480, 482, 484, 486, 489, 491, 494, 495, 497, 497, 498, 498, 499, 498, 498};

//global variables
word i, amplitudine_scalata, culoare, esantioane[nr_esant];
double real[nr_esant], imag[nr_esant], amplitudine_corectata_cu_dist;
boolean cont, card;

//------------------------------------------------------------------------
void setup()
{
  Wire.begin();
  Wire.setClock(400000);
  tft.begin();
  tft.fillScreen(BLACK);
  tft.setRotation(1);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextDatum(TC_DATUM); // Centered
  tft.setFreeFont(FSB12);
  xpos = tft.width() / 2; // Middle screen
  ypos = (tft.height() / 2) - tft.fontHeight(GFXFF); // Middle screen

  pinMode(button_up, INPUT_PULLUP);
  pinMode(button_OK, INPUT_PULLUP);
  pinMode(button_down, INPUT_PULLUP);
  pinMode(step_sensor, INPUT_PULLUP);

  tft.drawString("<c> Mirel Paun 2020", xpos, ypos, GFXFF);
  delay(1000);
  tft.fillScreen(BLACK);

  if (!digitalRead(buton_OK))
  {
    card = false; //do not store on SD card
    while (!digitalRead(buton_OK)) ;
  }
  else
  {

    if (SD.begin(SDC_CS))
    {
      card = true;
      Serial.begin(115200);
      delay(1000);
      tft.drawString("Waiting for PC to connect ...", xpos, ypos, GFXFF);
      tft.drawString("Press OK to continue ...", xpos, ypos + tft.fontHeight(GFXFF), GFXFF);
      while ((Serial.available() <= 0) && (digitalRead(buton_OK))) //wait for PC or OK button
      {
        delay(100);
      }
      if (Serial.available() > 0)
      {
        if (Serial.read() == 'A')
        {
          file = SD.open("Date.dat");
          if (file)
            //Send stored data
          {
            tft.fillScreen(BLACK);
            tft.drawString("Connected. Sending file!", xpos, ypos, GFXFF);
            while (file.available())
            {
              Serial.write(file.read());
            }
            file.close();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            tft.fillScreen(BLACK);
            tft.drawString("File sent!", xpos, ypos, GFXFF);
            tft.drawString("Press OK to delete file ...", xpos, ypos + tft.fontHeight(GFXFF), GFXFF);
            while (digitalRead(buton_OK)) //wait for OK button press
            {
              delay(100);
            }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////            
            tft.fillScreen(BLACK);
            tft.drawString("Deleting file ...", xpos, ypos, GFXFF);
            if (SD.remove("Date.dat")) {
              tft.drawString("Deleted!", xpos, ypos + tft.fontHeight(GFXFF), GFXFF);
            }
            else {
              tft.drawString("Error deleting file!", xpos, ypos + tft.fontHeight(GFXFF), GFXFF);
            }
            while (1) {
              ; // Stop
            }
          }
          else
          {
            tft.fillScreen(BLACK);
            tft.drawString("Error! File missing!", xpos, ypos, GFXFF);
            while (1) {
              ; // Stop
            }
          }
        }
      }
    }
    else
    {
      card = false; //no SD card
      tft.drawString("SD Card missing!!!", xpos, ypos, GFXFF);
      delay(2000);
    }
    tft.fillScreen(BLACK);
  }

  //DAC at 0
  Wire.beginTransmission(Adresa_MCP4725);
  Wire.write( 0 );        //MSB
  Wire.write( 0 );        //LSB
  Wire.endTransmission();
  delay(10);

  //Propagation velocities menu ------------------------------------------------------------
  while (!digitalRead(button_OK)) ;
  ypos = 10;
  tft.setFreeFont(FSB12);
  tft.drawString("Select propagation velocity c:", xpos, ypos, GFXFF);
  ypos += tft.fontHeight(GFXFF);
  tft.setFreeFont(FSB9);
  for (i = 0; i < (sizeof(medii_propagare) / sizeof(medii_propagare[0])); i++)
  {
    tft.drawString(medii_propagare[i], xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);
  }
  i = 0;
  ypos = 34 + i * tft.fontHeight(GFXFF);
  tft.drawRect(90, ypos, 300, 22, RED); //Draw cursor
  cont = false;
  do {
    while (digitalRead(button_up) && digitalRead(button_down) && digitalRead(button_OK)) {
      ; // wait
    }
    delay(10);
    if (!digitalRead(button_down))
    {
      if (i < (sizeof(medii_propagare) / sizeof(medii_propagare[0]) - 1)) {
        i++;
      }
      else {
        i = 0;
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_up))
    {
      if (i > 0) {
        i--;
      }
      else {
        i = (sizeof(medii_propagare) / sizeof(medii_propagare[0]) - 1);
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_OK))
    {
      c = viteze[i];
      cont = true;
    }
  } while (!cont);
  rezolutie = c / (2.0 * banda); //depth resolution [m]
  min_adanc = -4 * rezolutie;  	 //offset cables + antennas (aprox. 4 * resolution)

  if (card)
  {
    //Verify file existance, if missing, create it and write speed index
    if (!SD.exists("Date.dat")) {
      file = SD.open("Date.dat", FILE_WRITE);
      file.write(highByte(i));
      file.write(lowByte(i));
      file.close();
    }
  }

  //Depth menu--------------------------------------------------------------
  i = 0;
  word ind_adanc_max = sizeof(adancimi) / sizeof(adancimi[0]) - 1;

  tft.fillScreen(BLACK);
  while (!digitalRead(button_OK)) ;
  ypos = 10;
  tft.setFreeFont(FSB12);
  tft.drawString("Select maximum displayed depth:", xpos, ypos, GFXFF);
  ypos += tft.fontHeight(GFXFF);
  tft.setFreeFont(FSB9);
  for (i = 0; i <= ind_adanc_max; i++)
  {
    double adancime_temp = adancimi[i] * rezolutie + min_adanc;
    double zecimale = adancime_temp - floor(adancime_temp);
    String sir;
    if (zecimale < (adancime_temp / 10)) sir = String(adancime_temp, 0) + " m";
    else sir = String(adancime_temp, 1) + " m";
    char text[6];
    sir.toCharArray(text, 6);
    tft.drawString(text, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);
  }
  i = 0;
  ypos = 34 + i * tft.fontHeight(GFXFF);
  tft.drawRect(90, ypos, 300, 22, RED); //Draw cursor
  cont = false;
  delay(1000);
  do {
    while (digitalRead(button_up) && digitalRead(button_down) && digitalRead(button_OK)) {
      ; // wait
    }
    delay(10);
    if (!digitalRead(button_down))
    {
      if (i < ind_adanc_max) {
        i++;
      }
      else {
        i = 0;
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_up))
    {
      if (i > 0) {
        i--;
      }
      else {
        i = ind_adanc_max;
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_OK))
    {
      nr_cel_rez_vert = adancimi[i];
      max_adanc = nr_cel_rez_vert * rezolutie + min_adanc;
      double pas_adanc_temp = (max_adanc - min_adanc) / 4;
      i = 0;
      while ((abs(pas_adanc_temp - pasi_adanc[i]) > abs(pas_adanc_temp - pasi_adanc[i + 1])) && ((i + 1) < (sizeof(pasi_adanc) / sizeof(pasi_adanc[0]) - 1)))
      {
        i++;
      }
      if (abs(pas_adanc_temp - pasi_adanc[i]) > abs(pas_adanc_temp - pasi_adanc[i + 1])) i++;
      pas_adanc = pasi_adanc[i];
      nr_zecimale_pasi_adanc = zecimale_pasi_adanc[i];
      cont = true;
    }
  } while (!cont);

  //Distance menu--------------------------------------------------------------
  tft.fillScreen(BLACK);
  while (!digitalRead(button_OK)) ;
  ypos = 10;
  tft.setFreeFont(FSB12);
  tft.drawString("Select horizontal distance on screen:", xpos, ypos, GFXFF);
  ypos += tft.fontHeight(GFXFF);
  tft.setFreeFont(FSB9);
  for (i = 0; i < (sizeof(distante) / sizeof(distante[0])); i++)
  {
    String sir = String(distante[i]) + " m";
    char text[5];
    sir.toCharArray(text, 5);
    tft.drawString(text, xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);
  }
  i = 0;
  ypos = 34 + i * tft.fontHeight(GFXFF);
  tft.drawRect(90, ypos, 300, 22, RED); //Draw cursor
  cont = false;
  delay(1000);
  do {
    while (digitalRead(button_up) && digitalRead(button_down) && digitalRead(button_OK)) {
      ; // wait
    }
    delay(10);
    if (!digitalRead(button_down))
    {
      if (i < (sizeof(distante) / sizeof(distante[0]) - 1)) {
        i++;
      }
      else {
        i = 0;
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_up))
    {
      if (i > 0) {
        i--;
      }
      else {
        i = (sizeof(distante) / sizeof(distante[0]) - 1);
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_OK))
    {
      max_dist = distante[i];
      pas_dist = max_dist / 6;
      cont = true;
    }
  } while (!cont);

  //graph parameters
  inaltime_cel_rez = inaltime_grafic / nr_cel_rez_vert;  	  				  //[pixels]
  nr_cel_rez_oriz = max_dist / pas_deplasare_GPR;
  latime_cel_rez = latime_grafic / nr_cel_rez_oriz;                   //[pixels]
  // Draw grid
  tft.fillScreen(BLACK);
  Graph(tft, orig_x, orig_y, latime_grafic, inaltime_grafic, 0, max_dist, pas_dist, min_adanc, max_adanc, pas_adanc, "GPR", "Distance [m]", "Depth [m]");
  afis_card();
  check_displ_temp();
}

//------------------------------------------------------------------------
void loop()
{
  while (digitalRead(step_sensor) == 0)
  {
    check_displ_batlevel();
  }
  while (digitalRead(step_sensor) == 1)
  {
    check_displ_batlevel();
  }
  // If screen is full, delete and start again
  if (((pas % nr_cel_rez_oriz) == 0) && (pas != 0))
  {
    word nr_ecran = pas / nr_cel_rez_oriz;
    tft.fillScreen(BLACK);
    Graph(tft, orig_x, orig_y, latime_grafic, inaltime_grafic, nr_ecran * max_dist, (nr_ecran + 1) * max_dist, pas_dist, min_adanc, max_adanc, pas_adanc, "GPR", "Distance [m]", "Depth [m]");
    afis_card();
    check_displ_batlevel();
  }
  // Generate modulation signal and sample radar echo
  for (i = 0; i <= 4080; i = i + 16) // DAC goes from 0 - 4.98V
  {
    // Write to DAC
    Wire.beginTransmission(Adresa_MCP4725);
    Wire.write( highByte(i) );        //MSB
    Wire.write( lowByte(i) );         //LSB
    Wire.endTransmission();
    // Read from ADC
    esantioane[i >> 4] = analogRead(pin_ADC); // >>4 means /16
  }
  //Bring DAC to 0
  Wire.beginTransmission(Adresa_MCP4725);
  Wire.write( 0 );        //MSB
  Wire.write( 0 );        //LSB
  Wire.endTransmission();

  //Store on SD Card
  if (card)
  {
    file = SD.open("Date.dat", FILE_WRITE);
    for (i = 0; i < nr_esant; i++)
    {
      file.write(highByte(esantioane[i]));
      file.write(lowByte(esantioane[i]));
      //file.flush();
    }
    file.close();
  }

  // Prepare data for FFT
  for (i = 0; i < nr_esant; i++)
  {
	//real[i] = (double)(esantioane[i]) - (double)(corectie[i]); // Load samples and correct antenna coupling
    real[i] = (double)(esantioane[i]) - 512.0; // Load samples and eliminate d.c.
    imag[i] = 0.0;                                // Delete imaginary part
  }
  // Compute FFT
  //FFT.Windowing(real, nr_esant, FFT_WIN_TYP_HAMMING, FFT_FORWARD); //Windowing
  FFT.Compute(real, imag, nr_esant, FFT_FORWARD); //FFT
  FFT.ComplexToMagnitude(real, imag, nr_esant);   //Compute FFT and store it in real
  //Draw one column
  for (i = 0; i < nr_cel_rez_vert; i++)
  {
    amplitudine_corectata_cu_dist = real[i] * exp(i * 0.025);   // distance correction
    amplitudine_scalata = (word)(amplitudine_corectata_cu_dist * 255.0 / amplitudine_maxima);
    if (amplitudine_scalata > 255) amplitudine_scalata = 255;
    culoare = (((amplitudine_scalata & 0b11111000) << 8) + ((amplitudine_scalata & 0b11111100) << 3) + (amplitudine_scalata >> 3));
    tft.fillRect(orig_x + 1 + pas % nr_cel_rez_oriz * latime_cel_rez, orig_y + 1 - inaltime_grafic + i * inaltime_cel_rez, latime_cel_rez, inaltime_cel_rez, culoare);
  }
  pas++;
}

// Grid drawing routine
//
// list of arguments
// &tft display object
// gx = graphic origin x (bottom left)
// gy = graphic origin y (bottom left)
// w = graphic width (width)
// h = graphic height (height)
// xlo = lower limit axis x
// xhi = upper limit x axis
// xinc = increase (step) x axis
// ylo = lower limit y axis
// yhi = upper limit y axis
// yinc = increase (step) y axis
// title = graphic title
// xlabel = x-axis label
// ylabel = y axis label

void Graph(TFT_HX8357 &tft, double gx, double gy, double w, double h,
           double xlo, double xhi, double xinc,
           double ylo, double yhi, double yinc,
           char *title, char *xlabel, char *ylabel)
{
  double ydiv, xdiv;
  double i;
  double temp;
  int rot, newrot;

  unsigned int gcolor = DKBLUE; //grid color
  unsigned int acolor = RED;    //axis color
  unsigned int tcolor = WHITE;  //text color
  unsigned int bcolor = BLACK;  //background color

  tft.setFreeFont(NULL); //------------------------
  tft.setTextDatum(MR_DATUM);

  // draw x axis
  tft.drawLine(gx, gy - h, gx + w, gy - h, acolor);  //red axis
  tft.setTextColor(acolor, bcolor);
  tft.drawString(xlabel, (int)(gx + w) , (int)(gy - h) - 5, 2);
  // draw origin
  tft.setTextColor(tcolor, bcolor);
  tft.drawFloat(ylo, 3, gx - 4, gy - h, 1);

  for (i = 0; i <= yhi; i += yinc)
  {
    temp =  gy - h + (i - ylo) * h / (yhi - ylo);
    tft.drawLine(gx, temp, gx + w, temp, gcolor);
    // draw y axis labels
    tft.setTextColor(tcolor, bcolor);
    tft.drawFloat(i, nr_zecimale_pasi_adanc, gx - 4, temp, 1);
  }
  tft.drawLine(gx, gy, gx + w, gy, gcolor);//graph bottom line

  // draw y axis
  for (i = xlo; i <= xhi; i += xinc)
  {
    temp =  (i - xlo) * w / (xhi - xlo) + gx;
    if (i == xlo) 									//red axis
    {
      tft.drawLine(temp, gy, temp, gy - h, acolor);
      tft.setTextColor(acolor, bcolor);
      tft.setTextDatum(BC_DATUM);
      tft.drawString(ylabel, (int)temp, (int)(gy - h - 8) , 2);
    }
    else
    {
      tft.drawLine(temp, gy, temp, gy - h + 1, gcolor);
    }
    //draw x axis labels
    tft.setTextColor(tcolor, bcolor);
    tft.setTextDatum(TC_DATUM);
    tft.drawNumber(i, temp, gy + 7, 1);
  }

  //draw graph label
  tft.setTextColor(tcolor, bcolor);
  tft.drawString(title, (int)(gx + w / 2) , (int)(gy - h - 30), 4);
}

void afis_card()
{
  if (card)
  {
    tft.setTextColor(GREEN, BLACK);
    tft.drawString("CARD OK", 370, 2, 2);
  }
  else
  {
    tft.setTextColor(RED, BLACK);
    tft.drawString("NO CARD", 370, 2, 2);
  }
}

void check_displ_batlevel(void)
{
  //Measure and display battery state
  word val = analogRead(pin_BAT);
  if (val <= battery_level[0]) // low
  {
    tft.drawRect(430, 5, 20, 12, RED);
    tft.drawRect(431, 6, 18, 10, RED);
    tft.fillRect(450, 7, 3, 8, RED);
    tft.fillRect(432, 7, 16, 8, BLACK);
  }
  else if (val <= battery_level[1]) // med1
  {
    tft.drawRect(430, 5, 20, 12, WHITE);
    tft.fillRect(450, 7, 3, 8, WHITE);
    tft.fillRect(432, 7, 5, 8, WHITE);
    tft.fillRect(437, 7, 11, 8, BLACK);
  }
  else if (val <= battery_level[2]) // med2
  {
    tft.drawRect(430, 5, 20, 12, WHITE);
    tft.fillRect(450, 7, 3, 8, WHITE);
    tft.fillRect(432, 7, 11, 8, WHITE);
    tft.fillRect(443, 7, 5, 8, BLACK);
  }
  else // max
  {
    tft.fillRect(430, 5, 20, 12, WHITE);
    tft.fillRect(450, 7, 3, 8, WHITE);
  }
}
