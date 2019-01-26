#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <Adafruit_SSD1331.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

// screen pins
#define CLK 11
#define DIN 12
#define CS  10
#define RST  8
#define DC   9

#define MH_Z19_RX 7
#define MH_Z19_TX 6
#define BEEP_PIN  5

// http://www.barth-dev.de/online/rgb565-color-picker/
#define BLACK      0x0000
#define BLUE       0x001F
#define RED        0xF800
#define GREEN      0x07E0
#define CYAN       0x07FF
#define MAGENTA    0xF81F
#define YELLOW     0xFFE0  
#define WHITE      0xFFFF
#define LGRAY      0xCE79
#define MGRAY      0x632C
#define DGRAY      0x3186
const unsigned int COLOR_LVL [] = {0x0660, 0xCE79, 0xFE60, 0xFB20, 0xF800};

#define INTERVAL 5 // measurement interval, sec
#define PREHEAT 180 // preheat time, sec
#define MAX_DATA_ERRORS 15 //max of errors, reset after them
#define SCR_W 96 // screen width
#define BUF_SZ 5 // size of ring buffer for median filter
#define GRAPH_ADVANCE_MEASUREMENTS 30  // graph advances each this of measurements
#define BEEP_MIN_INTERVAL_SEC 300

#define FONT_W 6
#define FONT_H 7

#define GRAPH_H 20
#define CAT_X 60
#define CAT_Y 40
#define CAT_FACE_X 4
#define CAT_FACE_Y 5
#define CAT_FACE_W 13
#define CAT_FACE_H 8

const unsigned char bmp_body [] PROGMEM = {
  0x2,0x1,0x0,0x0,0x0,0x7,0x3,0x80,0x0,0x0,0xf,0x87,0xcf,0xe0,0x0,0xf,0xff,0xff,0xf8,0x0,
  0x1f,0xff,0xff,0xfe,0x0,0x1f,0xff,0xff,0xff,0x0,0x1f,0xff,0xff,0xff,0x0,0x1f,0xff,0xff,0xff,0x80,
  0x1f,0xff,0xff,0xff,0x80,0x1f,0xff,0xff,0xff,0x80,0x1f,0xff,0xff,0xff,0x80,0x1f,0xff,0xff,0xff,0x80,
  0x1f,0xff,0xff,0xff,0x80,0x1f,0xff,0xff,0xff,0x80,0xf,0xff,0xff,0xfb,0x80,0xf,0xff,0xff,0xfb,0x0,
  0x7,0xff,0xfe,0x7,0x0,0x3,0xff,0xfd,0xfe,0x0,0x1,0xff,0xfd,0xf8,0x0
};

const unsigned char bmp_stripes [] PROGMEM = {
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x3,0x20,0x0,0x0,0xa8,0x3,0x30,0x0,
  0x0,0xa8,0x18,0x30,0x0,0xc0,0x0,0x20,0x0,0x0,0x20,0x0,0x20,0x0,0x0,0x20,0x0,0x10,0x0,0x0,
  0x40,0x0,0x8,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,
  0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x2,0x0,0x0,0x0,0x0,0x3,0x0,
  0x0,0x0,0x0,0x1,0x0,0x0,0x0,0x1,0x98,0x0,0x0,0x0,0x1,0x98,0x0
};

const unsigned char bmp_face0_0 [] PROGMEM = {0x0,0x0,0x40,0x10,0xa0,0x28,0xa,0x80,0x5,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
const unsigned char bmp_face0_1 [] PROGMEM = {0x0,0x0,0x20,0x8,0x0,0x0,0x0,0x0,0x8,0x80,0x0,0x0,0x0,0x0,0x0,0x0};

const unsigned char bmp_face1_0 [] PROGMEM = {0x0,0x0,0x20,0x20,0x20,0x20,0xa,0x80,0x5,0x0,0x0,0x0,0x0,0x0,0x0,0x0};
const unsigned char bmp_face1_1 [] PROGMEM = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x8,0x80,0x0,0x0,0x0,0x0,0x0,0x0};

const unsigned char bmp_face2_0 [] PROGMEM = {0x0,0x0,0x20,0x20,0x20,0x20,0x2,0x0,0x5,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

const unsigned char bmp_face3_0 [] PROGMEM = {0x0,0x0,0x40,0x10,0x20,0x20,0x42,0x10,0x7,0x0,0x0,0x0,0x0,0x0,0x0,0x0};

const unsigned char bmp_face4_0 [] PROGMEM = {0x0,0x0,0x0,0x0,0x0,0x0,0x20,0x20,0x0,0x0,0x0,0x0,0x7,0x0,0x0,0x0};
const unsigned char bmp_face4_1 [] PROGMEM = {0x0,0x0,0x70,0x70,0x88,0x88,0x88,0x88,0x88,0x88,0x72,0x70,0x0,0x0,0x5,0x0};
const unsigned char bmp_face4_2 [] PROGMEM = {0x0,0x0,0x0,0x0,0x70,0x70,0x50,0x50,0x70,0x70,0x0,0x0,0x0,0x0,0x0,0x0};

int buf[BUF_SZ];
int ppm = 0;
int previous_ppm = 0;
long avg_ppm_past_period = 0;
int prev_lvl = -1;
int errorCount = 0;
unsigned int measure_id = -1;
unsigned int graphic_pos = 0;
unsigned int graph_advance_ticks = 0;
unsigned long last_beep_time_ms = 0;
SoftwareSerial co2Serial(MH_Z19_RX, MH_Z19_TX); // define MH-Z19
Adafruit_SSD1331 disp = Adafruit_SSD1331(CS, DC, DIN, CLK, RST); // define SSD1331 96x64 OLED

void(* resetFunc) (void) = 0; //declare reset function @ address 0

int readCO2() {
  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79}; // command to ask for data
  byte response[9];

  co2Serial.write(cmd, 9);
  co2Serial.readBytes(response, 9);
  measure_id += 1;
  if (response[0] != 0xFF) {
    Serial.println("Wrong starting byte from co2 sensor!");
    for (int i = 0; i < 9; i++) Serial.print(String(response[i], HEX) + " ");
    Serial.println("");
    return -1;
  }

  if (response[1] != 0x86) {
    Serial.println("Wrong command from co2 sensor!");
    for (int i = 0; i < 9; i++) Serial.print(String(response[i], HEX) + " ");
    Serial.println("");
    return -2;
  }

  int responseHigh = (int) response[2];
  int responseLow = (int) response[3];
  int ppm = (256 * responseHigh) + responseLow;
  buf[measure_id % BUF_SZ] = ppm;
  if (measure_id < BUF_SZ) {
    return ppm;
  } else {
    return median_ppm();
  }
}

int median_ppm() {
  int buf_copy[BUF_SZ];
  memcpy(buf_copy, buf, BUF_SZ);
  sort(buf_copy, BUF_SZ);
  return buf_copy[BUF_SZ / 2];
}

void sort(int arr[], int n) {
  int i, key, j;
  for (i = 1; i < n; i++) {
    key = arr[i];

    for (j = i - 1; j >= 0 && arr[j] > key; j -= 1) {
      arr[j + 1] = arr[j];
    }
    arr[j + 1] = key;
  }
}

void wait_sec(unsigned int sec) {
  unsigned int sec_passed = 0;
  unsigned int msec_acc = 0;
  unsigned long previousMillis = millis();
  
  while (sec_passed < sec) {
    if (msec_acc >= 1000) {
      unsigned int inc = msec_acc / 1000;
      sec_passed += inc;
      msec_acc -= inc * 1000;
    }
    delay(10);
    msec_acc += millis() - previousMillis;
    previousMillis = millis();
  }
}

void draw_bg() {
  disp.fillScreen(BLACK);
  disp.setTextWrap(false);
  disp.setTextSize(1);
  
  disp.setTextColor(WHITE);
  disp.setCursor(0, GRAPH_H + 6);
  disp.print("PPM = ");
  disp.drawFastHLine(0, GRAPH_H, 96, DGRAY);
  disp.drawBitmap(CAT_X, CAT_Y, bmp_body,    33, 19, MGRAY);
  disp.drawBitmap(CAT_X, CAT_Y, bmp_stripes, 33, 19, DGRAY);
}

byte ppm_to_lvl(int ppm) {
  if (ppm < 600) {
    return 0;
  } else if (ppm < 900) {
    return 1;
  } else if (ppm < 1200) {
    return 2;
  } else if (ppm < 1400) {
    return 3;
  }
  return 4;
}

void draw_ppm(int ppm) {
  int x = FONT_W * 6;
  int y = GRAPH_H + 6;
  unsigned int ppm_color = 0;
  int lvl = ppm_to_lvl(ppm);

  disp.setTextColor(BLACK);
  disp.setCursor(x, y);
  disp.print(String(previous_ppm));
  disp.setTextColor(COLOR_LVL[lvl]);
  disp.setCursor(x, y);
  disp.print(String(ppm));
  if (lvl != prev_lvl) {
    draw_cat_face(lvl);
  }
  prev_lvl = lvl;
}

void draw_graph(int ppm) {
  unsigned int x = (graphic_pos + 1) % SCR_W;

  int height = constrain(map(ppm, 400, 1400, 0, GRAPH_H), 0, GRAPH_H);
  disp.drawFastVLine(x, 0, GRAPH_H, BLACK);
  if (height > 0) disp.drawFastVLine(x, GRAPH_H - height, height, 0x0353);

  disp.drawFastVLine(graphic_pos, GRAPH_H + 1, 2, BLACK);
  disp.drawFastVLine(x, GRAPH_H + 1, 1, CYAN);
  graphic_pos = x;
}

void draw_cat_face(int face) {
  disp.fillRect(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, CAT_FACE_W, CAT_FACE_H, MGRAY);
  if (face == 0) {
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face0_0, CAT_FACE_W, CAT_FACE_H, BLACK);
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face0_1, CAT_FACE_W, CAT_FACE_H, DGRAY);
  } else if (face == 1) {
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face1_0, CAT_FACE_W, CAT_FACE_H, BLACK);
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face1_1, CAT_FACE_W, CAT_FACE_H, DGRAY);
  } else if (face == 2) {
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face2_0, CAT_FACE_W, CAT_FACE_H, BLACK);
  } else if (face == 3) {
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face3_0, CAT_FACE_W, CAT_FACE_H, BLACK);
  } else if (face == 4) {
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face4_0, CAT_FACE_W, CAT_FACE_H, BLACK);
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face4_1, CAT_FACE_W, CAT_FACE_H, DGRAY);
    disp.drawBitmap(CAT_X + CAT_FACE_X, CAT_Y + CAT_FACE_Y, bmp_face4_2, CAT_FACE_W, CAT_FACE_H, WHITE);
  }
}

void setup() {
  Serial.begin(115200); // Init console
  Serial.println("Setup started");

  co2Serial.begin(9600); //Init sensor MH-Z19(14)

  disp.begin();

  Serial.println("Waiting for sensor to init");
  Serial.println("Heating...");
  disp.fillScreen(BLACK);
  disp.setCursor(0, 0);
  disp.print("Full graph timespan is " + String(GRAPH_ADVANCE_MEASUREMENTS * SCR_W * INTERVAL / 60) + " minutes");
  disp.setCursor(0, 30);
  disp.print("Heating");
  wait_sec(PREHEAT);
  disp.fillScreen(BLACK);
  disp.setCursor(0,0);
  disp.print("Heated!");
  Serial.println("Setup finished");
  Serial.println("");
  draw_bg();
}

void loop() {
  if (errorCount > MAX_DATA_ERRORS) {
    disp.setTextColor(RED);
    disp.setCursor(2,40);
    disp.print("Too many errors");
    disp.setCursor(2,50);
    disp.print("Resetting");
    Serial.println("Too many errors, resetting");
    delay(2000);
    resetFunc();
  }

  previous_ppm = ppm;
  ppm = readCO2();
  avg_ppm_past_period += ppm;
  bool dataError = false;

  if (ppm < 100 || ppm > 6000) {
    Serial.println("PPM not valid: " + String(ppm));
    errorCount++;
  } else {
    Serial.println("PPM = " + String(ppm));
  }
  if (ppm != previous_ppm) {
    draw_ppm(ppm);
  }
  graph_advance_ticks += 1;
  if (graph_advance_ticks >= GRAPH_ADVANCE_MEASUREMENTS) {
    draw_graph(avg_ppm_past_period / GRAPH_ADVANCE_MEASUREMENTS);
    graph_advance_ticks = 0;
    avg_ppm_past_period = 0;
  }

  byte lvl = ppm_to_lvl(ppm);
  if (lvl >= 3 && last_beep_time_ms + BEEP_MIN_INTERVAL_SEC * 1000L < millis()) {
    last_beep_time_ms = millis();
    if (lvl == 3) {
      tone(BEEP_PIN, 1200, 200);
    } else if (lvl == 4) {
      tone(BEEP_PIN, 666, 500);
    }
  }

  Serial.println(" --- ");
  wait_sec(INTERVAL);
}

