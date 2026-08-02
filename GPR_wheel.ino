//////////////////////////////////
//     <c> Mirel Paun 2026      //
//////////////////////////////////

//ADC sampling period is 224 us
//stored file format - the first number (2 bytes) contains the velocity index, the following contain the time domain samples
#define MCP4725_Address 96            //DAC address
#define pin_ADC A15                   //ADC input pin
#define pin_BAT A13                   //Battery voltage input pin
#define WHITE     0xFFFF              //color
#define BLACK     0x0000              //color
#define RED       0xF800              //color
#define GREEN     0x07E0              //color
#define DKBLUE    0x000D              //color
#define button_up 7
#define button_OK 6
#define button_down 5
#define wheel_trigger 3
#define SDC_CS 53                     //SD card control pin
#define BaudRate 115200               //Serial port baud rate
#include <Wire.h>
#include "fft.h"
#include <TFT_HX8357.h>
#include "Free_Fonts.h"
#include <SD.h>

TFT_HX8357 tft = TFT_HX8357();

File file;

//GPR global parameters
const double GPR_horizontal_step = 0.12;  //GPR horizontal step [m]
const double bandwidth = 539000000;       //GPR bandwidth [Hz]; 355.7 MHz - 894.7 MHz
const word no_of_samples = 256;           //no. of time domain samples
const word cable_offset = 7;              //offset cables + antennas in no. of resolution cells

const word battery_level[3] = {384, 392, 401};   //battery voltage threshold levels: low - med1 - med2 (V_bat/5V*1023*0.4)
//modulator signal ramp that compensates JTOS-850VW+ nonlinearity
const word modulator_sig[no_of_samples] = {111, 119, 127, 134, 142, 150, 158, 166, 174, 182, 190, 198, 206, 214, 222, 230, 239, 248, 256, 265, 274, 282, 291, 299, 308, 317, 325, 334, 343, 351, 360, 369, 377, 386, 394, 403, 412, 420, 429, 438, 447, 456, 466, 475, 485, 494, 504, 513, 523, 532, 542, 551, 561, 570, 580, 589, 599, 608, 618, 627, 637, 646, 656, 665, 675, 685, 695, 705, 715, 725, 735, 745, 755, 765, 775, 785, 795, 805, 815, 825, 835, 845, 855, 865, 875, 885, 895, 906, 917, 928, 939, 950, 960, 971, 982, 993, 1004, 1015, 1026, 1036, 1047, 1058, 1069, 1080, 1091, 1102, 1113, 1125, 1137, 1149, 1161, 1172, 1184, 1196, 1208, 1220, 1232, 1244, 1256, 1268, 1280, 1292, 1303, 1315, 1327, 1340, 1353, 1366, 1379, 1392, 1405, 1418, 1431, 1444, 1457, 1470, 1483, 1496, 1509, 1522, 1535, 1548, 1562, 1576, 1590, 1605, 1619, 1633, 1647, 1661, 1675, 1689, 1704, 1718, 1732, 1746, 1760, 1775, 1790, 1806, 1821, 1837, 1853, 1868, 1884, 1899, 1915, 1931, 1946, 1962, 1977, 1993, 2010, 2027, 2044, 2061, 2077, 2094, 2111, 2128, 2145, 2162, 2179, 2196, 2213, 2232, 2251, 2270, 2289, 2308, 2328, 2347, 2366, 2385, 2404, 2423, 2443, 2462, 2481, 2500, 2519, 2538, 2558, 2577, 2596, 2615, 2634, 2654, 2675, 2697, 2720, 2742, 2764, 2786, 2808, 2830, 2853, 2875, 2899, 2923, 2948, 2972, 2996, 3021, 3045, 3069, 3094, 3120, 3147, 3174, 3201, 3228, 3255, 3282, 3308, 3337, 3367, 3396, 3426, 3456, 3486, 3516, 3546, 3578, 3611, 3643, 3676, 3708, 3741, 3774, 3809, 3844, 3880, 3915, 3950, 3985};

//menus
const char *propagation_media[] = {"Air - 0.3 m/ns", "Ice - 0.16 m/ns", "Dry sand - 0.15 m/ns", "Dry soil (Granite) - 0.13 m/ns", "Limestone - 0.12 m/ns", "Asphalt - 0.11 m/ns", "Concrete - 0.1 m/ns", "Moist soil - 0.09 m/ns", "Wet soil (Silt) - 0.07 m/ns", "Saturated sand (Clay) - 0.06 m/ns", "Sweet water - 0.03 m/ns", "Salt water - 0.01 m/ns"};
const double velocities[] = {300000000, 160000000, 150000000, 130000000, 120000000, 110000000, 100000000, 90000000, 70000000, 60000000, 30000000, 10000000};
const word depths[] = {16, 32, 64, 128}; //resolution multiples
const double depth_steps[] = {10, 5, 2.5, 1, 0.5, 0.25, 0.1, 0.05, 0.025, 0.01};
const word depth_steps_decimal_places[] =  {0,  0, 1  , 0,   1,    2,   1,    2,     3,    2};
const word distances[] = {6, 12, 24, 48};
const unsigned long inf_lim_max_amplitude = 1000, sup_lim_max_amplitude = 40000;

//graph constants
const word orig_x = 60, orig_y = 290, graph_width = 400, graph_height = 256; //graph origin coord. and dimensions [pixels]
//graph global variables
double max_dist, dist_step, max_depth, min_depth, depth_step, resolution; // [m]
double c; //[m/s]
word no_res_cells_horiz, no_res_cells_vert, res_cell_height, res_cell_width, interp_order_depth, no_depth_steps_decimal_places, xpos, ypos, scan_index = 0;

//global variables
word i, j, scaled_amplitude, color, M_EXPONENT, correction[no_of_samples / 2  * 2];  //last 2 = maximum depth interpolation order
short sig_r[no_of_samples * 2];             //2 = maximum depth interpolation order
short sig_i[no_of_samples * 2];             //2 = maximum depth interpolation order
unsigned long max_amplitude = 12000;        //maximum FFT magnitude value (for scaling)
unsigned long depth_corrected_amplitude;
boolean cont, card, file_existance;

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
  xpos = tft.width() / 2; // Middle of screen
  ypos = (tft.height() / 2) - tft.fontHeight(GFXFF); // Middle of screen

  pinMode(button_up, INPUT_PULLUP);
  pinMode(button_OK, INPUT_PULLUP);
  pinMode(button_down, INPUT_PULLUP);
  pinMode(wheel_trigger, INPUT);

  tft.drawString("<c> Mirel Paun 2026", xpos, ypos, GFXFF);
  delay(1000);
  tft.fillScreen(BLACK);

  if (!digitalRead(button_OK))
  {
    card = false; //do not store on SD card
    file_existance = false; //no file
    while (!digitalRead(button_OK)) ;
  }
  else
  {

    if (SD.begin(SDC_CS))
    {
      card = true;
      if (SD.exists("Data.gpr"))
      {
        file_existance = true;
        Serial.begin(BaudRate);
        delay(1000);
        tft.drawString("There is data on SD card!", xpos, ypos, GFXFF);
        tft.drawString("Waiting for PC to connect ...", xpos, ypos + tft.fontHeight(GFXFF), GFXFF);
        tft.drawString("Press OK to continue ...", xpos, ypos + 2 * (tft.fontHeight(GFXFF)), GFXFF);
        while ((Serial.available() <= 0) && (digitalRead(button_OK))) //wait for PC or OK button
        {
          delay(100);
        }
        if (Serial.available() > 0)
        {
          if (Serial.read() == 'A')
          {
            file = SD.open("Data.gpr");
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
              tft.fillScreen(BLACK);
              tft.drawString("File sent!", xpos, ypos, GFXFF);
              tft.drawString("Press OK to delete file ...", xpos, ypos + tft.fontHeight(GFXFF), GFXFF);
              while (digitalRead(button_OK)) //wait for OK button press
              {
                delay(100);
              }
              tft.fillScreen(BLACK);
              tft.drawString("Deleting file ...", xpos, ypos, GFXFF);
              if (SD.remove("Data.gpr")) {
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
        file_existance = false;
      }
    }
    else
    {
      card = false; //no SD card
      file_existance = false; //no file
      tft.drawString("SD Card missing!!!", xpos, ypos, GFXFF);
      delay(2000);
    }
    tft.fillScreen(BLACK);
  }

  //Bring DAC at start
  Wire.beginTransmission(MCP4725_Address);
  Wire.write( highByte(modulator_sig[0]) );     //MSB
  Wire.write( lowByte(modulator_sig[0]) );      //LSB
  Wire.endTransmission();
  delay(10);

  if ((!card) || (!file_existance))
  {
    //Propagation velocities menu ------------------------------------------------------------
    while (!digitalRead(button_OK)) ;
    ypos = 10;
    tft.setFreeFont(FSB12);
    tft.drawString("Select propagation velocity c:", xpos, ypos, GFXFF);
    ypos += tft.fontHeight(GFXFF);
    tft.setFreeFont(FSB9);
    for (i = 0; i < (sizeof(propagation_media) / sizeof(propagation_media[0])); i++)
    {
      tft.drawString(propagation_media[i], xpos, ypos, GFXFF);
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
        if (i < (sizeof(propagation_media) / sizeof(propagation_media[0]) - 1)) {
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
          i = (sizeof(propagation_media) / sizeof(propagation_media[0]) - 1);
        }
        tft.drawRect(90, ypos, 300, 22, BLACK);
        ypos = 34 + i * tft.fontHeight(GFXFF);
        tft.drawRect(90, ypos, 300, 22, RED);
        delay(500);
      }
      else if (!digitalRead(button_OK))
      {
        c = velocities[i];
        cont = true;
      }
    } while (!cont);
    resolution = c / (2.0 * bandwidth);                  //depth resolution [m]
    min_depth = -1.0 * float(cable_offset) * resolution; //offset cables + antennas

    if (card)
    {
      //Verify file existance, if missing, create it and write velocity index
      if (!file_existance) {
        file = SD.open("Data.gpr", FILE_WRITE);
        file.write(highByte(i));
        file.write(lowByte(i));
        file.close();
      }
    }
  }
  else
  {
    //Read velocity index from file
    file = SD.open("Data.gpr", FILE_READ);
    i = (file.read()) << 8;  //highbyte
    i = i | (file.read());   //lowbyte
    file.close();
    c = velocities[i];
    resolution = c / (2.0 * bandwidth);                  //depth resolution [m]
    min_depth = -1.0 * float(cable_offset) * resolution; //offset cables + antennas
  }

  //Depth menu--------------------------------------------------------------
  i = 0;
  word max_depth_index = sizeof(depths) / sizeof(depths[0]) - 1;

  tft.fillScreen(BLACK);
  while (!digitalRead(button_OK)) ;
  ypos = 10;
  tft.setFreeFont(FSB12);
  tft.drawString("Select maximum displayed depth:", xpos, ypos, GFXFF);
  ypos += tft.fontHeight(GFXFF);
  tft.setFreeFont(FSB9);
  for (i = 0; i <= max_depth_index; i++)
  {
    double temp_depth = depths[i] * resolution + min_depth;
    double no_of_decimal_places = temp_depth - floor(temp_depth);
    String temp_str;
    if (no_of_decimal_places < (temp_depth / 10)) temp_str = String(temp_depth, 0) + " m";
    else temp_str = String(temp_depth, 1) + " m";
    char text[6];
    temp_str.toCharArray(text, 6);
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
      if (i < max_depth_index) {
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
        i = max_depth_index;
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_OK))
    {
      no_res_cells_vert = depths[i];
      max_depth = no_res_cells_vert * resolution + min_depth;
      double depth_step_temp = (max_depth - min_depth) / 4;
      i = 0;
      while ((abs(depth_step_temp - depth_steps[i]) > abs(depth_step_temp - depth_steps[i + 1])) && ((i + 1) < (sizeof(depth_steps) / sizeof(depth_steps[0]) - 1)))
      {
        i++;
      }
      if (abs(depth_step_temp - depth_steps[i]) > abs(depth_step_temp - depth_steps[i + 1])) i++;
      depth_step = depth_steps[i];
      no_depth_steps_decimal_places = depth_steps_decimal_places[i];
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
  for (i = 0; i < (sizeof(distances) / sizeof(distances[0])); i++)
  {
    String temp_str = String(distances[i]) + " m";
    char text[5];
    temp_str.toCharArray(text, 5);
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
      if (i < (sizeof(distances) / sizeof(distances[0]) - 1)) {
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
        i = (sizeof(distances) / sizeof(distances[0]) - 1);
      }
      tft.drawRect(90, ypos, 300, 22, BLACK);
      ypos = 34 + i * tft.fontHeight(GFXFF);
      tft.drawRect(90, ypos, 300, 22, RED);
      delay(500);
    }
    else if (!digitalRead(button_OK))
    {
      max_dist = distances[i];
      dist_step = max_dist / 6;
      cont = true;
    }
  } while (!cont);

  //graph parameters
  res_cell_height = graph_height / no_res_cells_vert;  	  			      //[pixels]
  no_res_cells_horiz = max_dist / GPR_horizontal_step;
  res_cell_width = graph_width / no_res_cells_horiz;                  //[pixels]

  interp_order_depth = 2;              // interpolation order for depth axis
  M_EXPONENT = 9;                      // FFT M_EXPONENT = log2(no_of_samples * interp_order_depth)

  // Create depth correction array
  for (i = 0; i < no_of_samples / 2 * interp_order_depth && i < 60 * interp_order_depth; i++)
  {
    correction[i] = round(100 * exp(i * 0.025 / interp_order_depth)); // depth correction
  }

  // Draw grid
  tft.fillScreen(BLACK);
  Graph(tft, orig_x, orig_y, graph_width, graph_height, 0, max_dist, dist_step, min_depth, max_depth, depth_step, "GPR", "Distance [m]", "Depth [m]");
  show_card_status();
  check_displ_batlevel();
}

//------------------------------------------------------------------------
void loop()
{
  // Wait for step wheel signal
  while (digitalRead(wheel_trigger) == 0)
  {
    check_displ_batlevel();
    show_card_status();

    //REF_LVL Menu--------------------------------------------------------
    if (!digitalRead(button_down))
    {
      if (max_amplitude > inf_lim_max_amplitude) {
        max_amplitude = max_amplitude - 1000;
      }
      else {
        max_amplitude = inf_lim_max_amplitude;
      }
      delay(500);
    }
    else if (!digitalRead(button_up))
    {
      if (max_amplitude < sup_lim_max_amplitude) {
        max_amplitude = max_amplitude + 1000;
      }
      else {
        max_amplitude = sup_lim_max_amplitude;
      }
      delay(500);
    }

  }
  while (digitalRead(wheel_trigger) == 1)
  {
    check_displ_batlevel();
    show_card_status();

    //REF_LVL Menu--------------------------------------------------------
    if (!digitalRead(button_down))
    {
      if (max_amplitude > inf_lim_max_amplitude) {
        max_amplitude = max_amplitude - 1000;
      }
      else {
        max_amplitude = inf_lim_max_amplitude;
      }
      delay(500);
    }
    else if (!digitalRead(button_up))
    {
      if (max_amplitude < sup_lim_max_amplitude) {
        max_amplitude = max_amplitude + 1000;
      }
      else {
        max_amplitude = sup_lim_max_amplitude;
      }
      delay(500);
    }

  }
  // If screen is full, delete and start again
  if (((scan_index % no_res_cells_horiz) == 0) && (scan_index != 0))
  {
    word nr_ecran = scan_index / no_res_cells_horiz;
    tft.fillScreen(BLACK);
    Graph(tft, orig_x, orig_y, graph_width, graph_height, nr_ecran * max_dist, (nr_ecran + 1) * max_dist, dist_step, min_depth, max_depth, depth_step, "GPR", "Distance [m]", "Depth [m]");
    show_card_status();
    check_displ_batlevel();
  }
  // Generate modulation signal and sample radar echo
  for (i = 0; i < no_of_samples; i++)
  {
    j = modulator_sig[i];
    // Write at DAC
    Wire.beginTransmission(MCP4725_Address);
    Wire.write( highByte(j) );        //MSB
    Wire.write( lowByte(j) );         //LSB
    Wire.endTransmission();
    // Read from ADC
    sig_r[i] = ((analogRead(pin_ADC)));
  }
  //Bring DAC at start
  Wire.beginTransmission(MCP4725_Address);
  Wire.write( highByte(modulator_sig[0]) );        //MSB
  Wire.write( lowByte(modulator_sig[0]) );         //LSB
  Wire.endTransmission();

  //Store on SD Card
  if (card)
  {
    file = SD.open("Data.gpr", FILE_WRITE);
    for (i = 0; i < no_of_samples; i++)
    {
      file.write(highByte(sig_r[i]));
      file.write(lowByte(sig_r[i]));
      //file.flush();
    }
    file.close();
  }

  //Draw one column
  //unsigned long maximum=0; // FFT maximum (for calibrating display)
  // Prepare data for FFT
  //real part:
  for (i = 0; i < no_of_samples; i++)
  {
     short sig = sig_r[i] - 512;      // eliminate d.c.
     sig_r[i] = sig << 6;             // * 64 (make the signal use all bits)
  }
  for (i = no_of_samples; i < no_of_samples * interp_order_depth; i++)
  {
    sig_r[i] = 0;                          // clear the rest of the used sig_r vector
  }
  //imaginary part:
  for (i = 0; i < no_of_samples * interp_order_depth; i++)
  {
    sig_i[i] = 0;                          // clear the sig_i vector
  }
  unsigned long local_max_amplitude = max_amplitude / interp_order_depth;
  word interp_res_cell_height = res_cell_height / interp_order_depth;
  // Perform the Fast Fourier Transform
  // Data is modified "in-place"
  fix_fft(sig_r, sig_i, M_EXPONENT, 0);
  // Compute spectral magnitudes (Absolute values), compensate depth and draw
  for (i = 0; i < no_res_cells_vert * interp_order_depth; i++)
  {
    //Compute magnitude by alpha max plus beta min algorithm 
    short abs_r = abs(sig_r[i]);
    short abs_i = abs(sig_i[i]);
    short max_val = (abs_r > abs_i) ? abs_r : abs_i;
    short min_val = (abs_r < abs_i) ? abs_r : abs_i;
    unsigned long magnitude = max_val + ((unsigned long)(min_val * 3) >> 3);

    if (i < 60 * interp_order_depth) {
      depth_corrected_amplitude = magnitude * correction[i]; // depth correction
    }
    else {
      depth_corrected_amplitude = magnitude * 448;
    }

    //if(depth_corrected_amplitude > maximum) maximum = depth_corrected_amplitude; // search FFT maximum (for calibrating display)
    scaled_amplitude = (word)((depth_corrected_amplitude << 2) / local_max_amplitude);//aprox. depth_corrected_amplitude * 255 / (local_max_amplitude << 6)
    if (scaled_amplitude > 255) scaled_amplitude = 255;
    color = (((scaled_amplitude & 0b11111000) << 8) + ((scaled_amplitude & 0b11111100) << 3) + (scaled_amplitude >> 3));
    tft.fillRect(orig_x + 1 + scan_index % no_res_cells_horiz * res_cell_width, orig_y + 1 - graph_height + i * interp_res_cell_height, res_cell_width, interp_res_cell_height, color);
  }
  scan_index++;
}

// Grid drawing routine
//
// list of arguments
// &tft display object
// gx = graph origin x (bottom left)
// gy = graph origin y (bottom left)
// w = graph width (width)
// h = graph height (height)
// xlo = lower limit axis x
// xhi = upper limit x axis
// xinc = increase (step) x axis
// ylo = lower limit y axis
// yhi = upper limit y axis
// yinc = increase (step) y axis
// title = graph title
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
    tft.drawFloat(i, no_depth_steps_decimal_places, gx - 4, temp, 1);
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

void show_card_status()
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
