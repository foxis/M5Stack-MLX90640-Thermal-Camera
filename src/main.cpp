// update 3 :mirror image
// reverseScreen=true/false, turn=front camera, false=Selfie
/*
  Read the temperature pixels from the MLX90640 IR array
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14769

  This example initializes the MLX90640 and outputs the 768 temperature values
  from the 768 pixels.

  This example will work with a Teensy 3.1 and above. The MLX90640 requires some
  hefty calculations and larger arrays. You will need a microcontroller with 20,000
  bytes or more of RAM.

  This relies on the driver written by Melexis and can be found at:
  https://github.com/melexis/mlx90640-library

  Hardware Connections:
  Connect the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  to the Qwiic board
  Connect the male pins to the Teensy. The pinouts can be found here: https://www.pjrc.com/teensy/pinout.html
  Open the serial monitor at 9600 baud to see the output
*/
#include <M5Stack.h>
#include <Wire.h>

#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define TA_SHIFT 8 //Default shift for MLX90640 in open air

#define COLS 32
#define ROWS 24
#define INTERPOLATED_COLS 56
#define INTERPOLATED_ROWS 56

float pixels[COLS * ROWS];
float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];

byte speed_setting = 2 ; // High is 1 , Low is 2
//bool reverseScreen = false;
bool reverseScreen = true;
bool display_grid = false;
bool autoscale_temp = true;
bool menu = false;
bool menu_emissivity = false;
float emissivity;
int emissivity_index = 23;

paramsMLX90640 mlx90640;

//low range of the sensor (this will be blue on the screen)
float MINTEMP = 5; // For color mapping
float min_v = 24; //Value of current min temp
float min_cam_v = -40; // Spec in datasheet


//high range of the sensor (this will be red on the screen)
float MAXTEMP = 40; // For color mapping
float max_v = 35; //Value of current max temp
float max_cam_v = 380; // Spec in datasheet
float resetMaxTemp = 45;

float spot_v = 0;

TFT_eSprite img = TFT_eSprite(&M5.Lcd);  // Create Sprite object "img" with pointer to "tft" object

bool MLX90640_ok = false;
long loopTime, startTime, endTime, fps;

//the colors we will be using
const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };

#define EMISSIVITY_SIZE (sizeof(emissivity_table) / sizeof(emissivity_table[0]))
struct emissivity_struct {
	char * name;
	float value;
} emissivity_table[] = {
	{"Aluminium: anodised",	0.77},
	{"Aluminium: polished",	0.05},
	{"Asbestos: board",	0.96},
	{"Asbestos: fabric",	0.78},
	{"Asbestos: paper",	0.93},
	{"Asbestos: slate",	0.96},
	{"Brass: highly polished",	0.03},
	{"Brass: oxidized",	0.61},
	{"Brick: common",	(.81+.86)/2.0},
	{"Brick: common, red",	0.93},
	{"Brick: facing, red",	0.92},
	{"Brick: fireclay",	0.75},
	{"Brick: masonry",	0.94},
	{"Brick: red",	0.90},
	{"Carbon: candle soot",	0.95},
	{"Carbon: graphite",	0.98},
	{"Carbon: purified",	0.80},
	{"Cement:",	0.54},
	{"Charcoal: powder",	0.96},
	{"Chipboard: untreated",	0.90},
	{"Chromium: polished",	0.10},
	{"Clay: fired",	0.91},
	{"Concrete",	0.92},
	{"Concrete: dry",	0.95},
	{"Concrete: rough",	.92-.97},
	{"Copper: polished",	0.05},
	{"Copper: oxidized",	0.65},
	{"Enamel: lacquer",	0.90},
	{"Fabric: green",	0.88},
	{"Fabric: uncoloured",	0.87},
	{"Fibreglass",	0.75},
	{"Fibre brd: porous",	0.85},
	{"Fibre brd: hard",	0.85},
	{"Filler: white",	0.88},
	{"Firebrick",	0.68},
	{"Formica",	0.94},
	{"Galvanized Pipe",	0.46},
	{"Glass",	0.92},
	{"Glass: semitransparent",	0.97},
	{"Glass: frosted",	0.96},
	{"Glass: frosted",	0.70},
	{"Glass: polished plate",	0.94},
	{"Granite: natural surface",	0.96},
	{"Graphite: powder",	0.97},
	{"Gravel",	0.28},
	{"Gypsum",	0.08},
	{"Hardwood: across grain",	0.82},
	{"Hardwood: along grain",	(.68+.73)/2.0},
	{"Ice",	0.97},
	{"Iron: heavily rusted",	(.91-.96)/2.0},
	{"Lacquer: bakelite",	0.93},
	{"Lacquer: dull black",	0.97},
	{"Lampblack",	0.96},
	{"Limestone: natural surface",	0.96},
	{"Mortar",	0.87},
	{"Mortar: dry",	0.94},
	{"P.V.C.",	.91-.93},
	{"Paint: 9560 series",	1.00},
	{"Paint: aluminium",	0.45},
	{"Paint, oil: average",	0.94},
	{"Paint: oil, black, gloss",	0.92},
	{"Paint: oil, grey, flat",	0.97},
	{"Paint: plastic, black",	0.95},
	{"Paint: plastic, white",	0.84},
	{"Paper: black",	0.90},
	{"Paper: black, dull",	0.94},
	{"Paper: cardboard",	0.81},
	{"Paper: green",	0.85},
	{"Paper: red",	0.76},
	{"Paper: white",	0.68},
	{"Paper: white bond",	0.93},
	{"Paper: yellow",	0.72},
	{"Paper: tar",	0.92},
	{"Pipes: glazed",	0.83},
	{"Plaster",	(.86+.90)/2.0},
	{"Plaster: rough coat",	0.91},
	{"Plasterboard",	0.90},
	{"Plastic: acrylic",	0.94},
	{"Plastic: black",	0.95},
	{"Plastic: white",	0.84},
	{"Plastic paper: red",	0.94},
	{"Plastic paper: white",	0.84},
	{"Plexiglass: Perpex",	0.86},
	{"Plywood",	(.83+.98)/2.0},
	{"Plywood: dry",	0.82},
	{"Plywood: untreated",	0.83},
	{"Polypropylene",	0.97},
	{"Porcelain: glazed",	0.92},
	{"Quartz",	0.93},
	{"Redwood: wrought",	0.83},
	{"Redwood: unwrought",	0.84},
	{"Rubber",	0.95},
	{"Rubber: black",	0.97},
	{"Sand",	0.90},
	{"Skin, human",	0.98},
	{"Snow",	0.80},
	{"Soil: dry",	0.92},
	{"Soil: frozen",	0.93},
	{"Soil: w/water",	0.95},
	{"Stainless Steel",	0.59},
	{"Stainless Plate",	0.34},
	{"Steel: galvanized",	0.28},
	{"Steel: rolled freshly",	0.24},
	{"Styrofoam: insulation",	0.60},
	{"Tape: electrical",	0.97},
	{"Tape: masking",	0.92},
	{"Tile: asbestos",	0.94},
	{"Tile: glazed",	0.94},
	{"Tin: burnished",	0.05},
	{"Tin: sheet iron",	0.06},
	{"Varnish: flat",	0.93},
	{"Wallpaper: light grey",	0.85},
	{"Wallpaper: red",	0.90},
	{"Water:",	0.95},
	{"Water: distilled",	0.95},
	{"Water: ice, smooth",	0.96},
	{"Water: frost crystals",	0.98},
	{"Water: snow",	0.85},
	{"Wood: planed",	0.90},
	{"Wood: panelling",	0.87},
	{"Wood: dry",	0.86},
};

float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols, float *dest, uint8_t dest_rows, uint8_t dest_cols);
void drawgrid(float *p, uint8_t rows, uint8_t cols, float boxWidth, float boxHeight);
void drawpixels(float *p, uint8_t rows, uint8_t cols, float boxWidth, float boxHeight);
void infodisplay();
void MLX90640_init();
void displayrunning();


void setup()
{
  M5.begin();
	WiFi.mode(WIFI_OFF);
	btStop();

  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz

  M5.Lcd.setRotation(1);
	M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.setTextSize(2);
	M5.Lcd.setTextColor(TFT_WHITE);
	img.createSprite(32*7, 32*7);
	img.setTextColor(TFT_WHITE);
  img.setTextSize(1);
	MLX90640_init();

  for (int icol = 0; icol <= 255;  icol++)
  {
    M5.Lcd.drawFastHLine(img.width()+1, img.height() - img.height() * (icol / 255.0), 15, camColors[icol]);
  }

	if (autoscale_temp) {
		MINTEMP = max_cam_v;
		MAXTEMP = min_cam_v;
	}

	emissivity = emissivity_table[emissivity_index].value;

  infodisplay();
}


void loop()
{
  loopTime = millis();
  startTime = loopTime;
  ///////////////////////////////
  // Set Min Value - LongPress //
  ///////////////////////////////
  if (M5.BtnA.pressedFor(1000)) {
    MINTEMP = max(min_cam_v, MINTEMP - 5);
		infodisplay();
  }

  ///////////////////////////////
  // Set Min Value - SortPress //
  ///////////////////////////////
  if (M5.BtnA.wasPressed()) {
		if (!menu)
			MINTEMP = max(min_cam_v, MINTEMP - 1);
		else {
			if (menu_emissivity) {
				if (emissivity_index > 0)
					emissivity_index --;
			} else {
				display_grid = !display_grid;
				menu = false;
			}
		}
		infodisplay();
  }

  /////////////////////
  // Reset settings  //
  /////////////////////
  if (M5.BtnB.wasPressed()) {
		if (!menu) {
    	MINTEMP = min_v - 1;
    	MAXTEMP = max_v + 1;
		} else {
			if (!menu_emissivity)
				autoscale_temp = !autoscale_temp;
			else {
				emissivity = emissivity_table[emissivity_index].value;
			}
			menu = false;
		}
		infodisplay();
  }

  ////////////////
  // Power Off  //
  ////////////////
  if (M5.BtnB.pressedFor(1000)) {
		if (menu) {
			menu_emissivity = true;
		} else
		menu = true;
		infodisplay();
		delay(1000);
  }

  ///////////////////////////////
  // Set Max Value - LongPress //
  ///////////////////////////////
  if (M5.BtnC.pressedFor(1000)) {
		if (menu) {
			M5.Lcd.fillScreen(TFT_BLACK);
	    M5.Lcd.setTextColor(YELLOW, BLACK);
	    M5.Lcd.drawCentreString("Power Off...", 160, 80, 4);
	    delay(1000);
	    M5.powerOFF();
		}
		MAXTEMP = min(max_cam_v, MAXTEMP + 5);
		infodisplay();
  }

  ///////////////////////////////
  // Set Max Value - SortPress //
  ///////////////////////////////
  if (M5.BtnC.wasPressed()) {
		if (!menu)
			MAXTEMP = min(max_cam_v, MAXTEMP + 1);
		else {
			if (menu_emissivity) {
				if (emissivity_index < EMISSIVITY_SIZE - 1)
					emissivity_index ++;
			}
		}
		infodisplay();
  }

  M5.update();

	if (!menu) {
	  for (byte x = 0 ; x < speed_setting ; x++) // x < 2 Read both subpages
	  {
	    uint16_t mlx90640Frame[834];
	    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
	    if (status < 0)
	    {
	    }

	    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
	    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
	    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
	    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, pixels); //save pixels temp to array (pixels)
	  }

	  //Reverse image
	  if (reverseScreen == 1)
	  {
	    for (int y = 0 ; y < ROWS ; y++)
	    {
				float * p = pixels + y * COLS;
	      for (int x = 0; x < COLS / 2; x++)
	      {
					float tmp;
					tmp = p[x];
					p[x] = p[COLS - x - 1];
					p[COLS - x - 1] = tmp;
				}
	    }
	  }

		if (autoscale_temp) {
			max_v = min_cam_v;
		  min_v = max_cam_v;
		} else {
			max_v = MINTEMP;
		  min_v = MAXTEMP;
		}
	  spot_v = pixels[COLS / 2 - 1 + COLS * (ROWS / 2 - 1)];
		spot_v += pixels[COLS / 2 - 1 + COLS * ROWS / 2];
		spot_v += pixels[COLS / 2 + COLS * (ROWS / 2 - 1)];
		spot_v += pixels[COLS / 2 + COLS * ROWS / 2];
		spot_v /= 4.0f;

		float * p = pixels;
		for ( int y = 0; y < ROWS * COLS; y++ ) {
			max_v = max(max_v, *p);
			min_v = min(min_v, *p);
			++p;
		}

		if (autoscale_temp) {
			if (abs(min_v - MINTEMP) > 1)
				MINTEMP = min_v - 1;
			if (abs(max_v - MAXTEMP) > 1)
				MAXTEMP = max_v + 1;
		}

	  interpolate_image(pixels, ROWS, COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
		drawpixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS, img.width() / INTERPOLATED_COLS, img.height() / INTERPOLATED_ROWS);
		drawgrid(pixels, ROWS, COLS, img.width() / (float)COLS, img.height() / (float)ROWS);

		img.pushSprite(0, 0);

		if (autoscale_temp)
			infodisplay();
		displayrunning();
	}
}

/*** ***/
void MLX90640_init() {
	//Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];//32 * 24 = 768
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  //if (status != 0)
  //  Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  //if (status != 0)
  //  Serial.println("Parameter extraction failed");

  int SetRefreshRate;
  //Setting MLX90640 device at slave address 0x33 to work with 16Hz refresh rate:
  // 0x00 – 0.5Hz
  // 0x01 – 1Hz
  // 0x02 – 2Hz
  // 0x03 – 4Hz
  // 0x04 – 8Hz // OK
  // 0x05 – 16Hz // OK
  // 0x06 – 32Hz // Fail
  // 0x07 – 64Hz
  SetRefreshRate = MLX90640_SetRefreshRate (MLX90640_address, 0x04);
  //Once params are extracted, we can release eeMLX90640 array
}

/***infodisplay()*****/
void infodisplay(void) {
	const int x = img.width() + 16 + 3;
	const int w = M5.Lcd.width() - img.width() + 16;
	const int h = img.height();
	M5.Lcd.setTextSize(2);
  M5.Lcd.fillRect(x, 0, w, 16, TFT_BLACK); //Clear MaxTemp area
  M5.Lcd.setCursor(x, 0); //move to bottom right
  M5.Lcd.print(MAXTEMP , 1);  // update MAXTEMP
  M5.Lcd.setCursor(x, h - 16);  // update MINTEMP text
  M5.Lcd.fillRect(x, h - 16, w, 16, TFT_BLACK);
  M5.Lcd.print(MINTEMP , 1);
	M5.Lcd.setTextSize(1);
	M5.Lcd.setCursor(0, h+3);
	M5.Lcd.fillRect(0, h, M5.Lcd.width(), M5.Lcd.height()-h, TFT_BLACK); //Clear menu area
	if (!menu) {
		M5.Lcd.print("     Min adj          Settings       Max adj");
	} else {
		if (!menu_emissivity)
			M5.Lcd.print("     grid         auto scale      Power off");
		else {
			M5.Lcd.setCursor(32, h+3);
			M5.Lcd.print("<");
			M5.Lcd.setCursor(img.width(), h+3);
			M5.Lcd.print(">");
			M5.Lcd.setCursor(64, h+3);
			M5.Lcd.print(emissivity_table[emissivity_index].name);
		}
	}
}

void displayrunning() {
	const int x = img.width() + 16 + 3;
	const int h = img.height();
	const int w = M5.Lcd.width() - img.width() + 16;

	M5.Lcd.setTextSize(2);
	M5.Lcd.fillRect(x, h / 2 - 8, w, 16, TFT_BLACK); // clear spot temp text
  M5.Lcd.setTextColor(TFT_WHITE);

  if (max_v > max_cam_v || max_v < min_cam_v ) {
		M5.Lcd.setCursor(x, h / 2 - 8);      // update min & max temp
    M5.Lcd.setTextColor(TFT_RED);
    M5.Lcd.printf("Error", 1);
  }
  else
  {
		M5.Lcd.fillRect(x, h - 32, w, 16, TFT_BLACK); // clear min temp text
		M5.Lcd.setCursor(x, h - 32);      // update min temp
    M5.Lcd.print(min_v, 1);

		M5.Lcd.fillRect(x, 16, w, 16, TFT_BLACK);  // clear max temp text
		M5.Lcd.setCursor(x, 16);      // update max temp
    M5.Lcd.print(max_v, 1);

		M5.Lcd.setCursor(x, h / 2 - 8);      // update min & max temp
    M5.Lcd.print(spot_v, 1);
  }
  loopTime = millis();
  endTime = loopTime;
  fps = 1000 / (endTime - startTime);
  //M5.Lcd.fillRect(310, 209, 10, 12, TFT_BLACK); //Clear fps text area
  M5.Lcd.fillRect(x, h, w, M5.Lcd.height()-h, TFT_BLACK); //Clear fps text area
  M5.Lcd.setTextSize(1);
	M5.Lcd.setCursor(x+32, h+1);
  M5.Lcd.print("fps:" + String( fps ));
	M5.Lcd.setCursor(x+32, h+9);
  M5.Lcd.print("bat:" + String( M5.Power.getBatteryLevel() ));
}

void drawpixels(float *p, uint8_t rows, uint8_t cols, float boxWidth, float boxHeight) {
  for (int y = 0; y < rows; y++)
  {
    for (int x = 0; x < cols; x++)
    {
      float val = min(MAXTEMP, max(MINTEMP, *p));
			++p;

      uint8_t colorIndex = map((int)val, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);// 0 ~ 255
      //draw the pixels!
      img.fillRect(x * boxWidth, y * boxHeight, boxWidth, boxHeight, camColors[colorIndex]);
    }
  }
}

void drawgrid(float *p, uint8_t rows, uint8_t cols, float boxWidth, float boxHeight) {
	int grX = 3;
  int grY = 3;
	const float scale = 1 / (float)(grX * grY);

	if (display_grid) {
		for (int y = 0; y < rows; ) {
			int ty = boxHeight * y + boxHeight/3;
		  for (int x = 0; x < cols;) {
		    float val = 0;
		    for (int grx=0;grx<grX;grx++)
		      for (int gry=0;gry<grY;gry++)
		          val += get_point(p, (rows), (cols), (x+grx), (y+gry));
		    val = val * scale;

		    img.setCursor(x * boxWidth + boxWidth/3, ty); // update spot temp text
		    img.print(val, 0);
				x += grX;
		  }
			y += grY;
		}
	}
	else {
		int x = img.width() / 2;
		int y = img.height() / 2;
		img.drawCircle(x, y, boxWidth, TFT_WHITE);     // update center spot icon
    img.drawFastHLine(x-boxWidth, y, 2 * boxWidth, TFT_WHITE); // vertical line
    img.drawFastVLine(x, y-boxWidth, 2 * boxWidth, TFT_WHITE); // horizontal line
	}
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}
