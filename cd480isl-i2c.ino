#include <Wire.h>


const uint8_t Font5x7_full[][5] PROGMEM = {


    {0x00, 0x00, 0x00, 0x00, 0x00},  // (space)
    {0x00, 0x00, 0x5F, 0x00, 0x00},  // !
    {0x00, 0x07, 0x00, 0x07, 0x00},  // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14},  // //
    {0x24, 0x2A, 0x7F, 0x2A, 0x12},  // $
    {0x23, 0x13, 0x08, 0x64, 0x62},  // %
    {0x36, 0x49, 0x55, 0x22, 0x50},  // &
    {0x00, 0x05, 0x03, 0x00, 0x00},  // '
    {0x00, 0x1C, 0x22, 0x41, 0x00},  // (
    {0x00, 0x41, 0x22, 0x1C, 0x00},  // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08},  // *
    {0x08, 0x08, 0x3E, 0x08, 0x08},  // +
    {0x00, 0x50, 0x30, 0x00, 0x00},  // ,
    {0x08, 0x08, 0x08, 0x08, 0x08},  // -
    {0x00, 0x60, 0x60, 0x00, 0x00},  // .
    {0x20, 0x10, 0x08, 0x04, 0x02},  // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E},  // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00},  // 1
    {0x42, 0x61, 0x51, 0x49, 0x46},  // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31},  // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10},  // 4
    {0x27, 0x45, 0x45, 0x45, 0x39},  // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30},  // 6
    {0x01, 0x71, 0x09, 0x05, 0x03},  // 7
    {0x36, 0x49, 0x49, 0x49, 0x36},  // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E},  // 9
    {0x00, 0x36, 0x36, 0x00, 0x00},  // :
    {0x00, 0x56, 0x36, 0x00, 0x00},  // ;
    {0x00, 0x08, 0x14, 0x22, 0x41},  // <
    {0x14, 0x14, 0x14, 0x14, 0x14},  // =
    {0x41, 0x22, 0x14, 0x08, 0x00},  // >
    {0x02, 0x01, 0x51, 0x09, 0x06},  // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E},  // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E},  // A
    {0x7F, 0x49, 0x49, 0x49, 0x36},  // B
    {0x3E, 0x41, 0x41, 0x41, 0x22},  // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C},  // D
    {0x7F, 0x49, 0x49, 0x49, 0x41},  // E
    {0x7F, 0x09, 0x09, 0x01, 0x01},  // F
    {0x3E, 0x41, 0x41, 0x51, 0x32},  // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F},  // H
    {0x00, 0x41, 0x7F, 0x41, 0x00},  // I
    {0x20, 0x40, 0x41, 0x3F, 0x01},  // J
    {0x7F, 0x08, 0x14, 0x22, 0x41},  // K
    {0x7F, 0x40, 0x40, 0x40, 0x40},  // L
    {0x7F, 0x02, 0x04, 0x02, 0x7F},  // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F},  // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E},  // O
    {0x7F, 0x09, 0x09, 0x09, 0x06},  // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E},  // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46},  // R
    {0x46, 0x49, 0x49, 0x49, 0x31},  // S
    {0x01, 0x01, 0x7F, 0x01, 0x01},  // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F},  // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F},  // V
    {0x7F, 0x20, 0x18, 0x20, 0x7F},  // W
    {0x63, 0x14, 0x08, 0x14, 0x63},  // X
    {0x03, 0x04, 0x78, 0x04, 0x03},  // Y
    {0x61, 0x51, 0x49, 0x45, 0x43},  // Z
    {0x00, 0x00, 0x7F, 0x41, 0x41},  // {
    {0x02, 0x04, 0x08, 0x10, 0x20},  // "\"
    {0x41, 0x41, 0x7F, 0x00, 0x00},  // }
    {0x04, 0x02, 0x01, 0x02, 0x04},  // ^
    {0x40, 0x40, 0x40, 0x40, 0x40},  // _
    {0x00, 0x01, 0x02, 0x04, 0x00},  // `
    {0x20, 0x54, 0x54, 0x54, 0x78},  // a
    {0x7F, 0x48, 0x44, 0x44, 0x38},  // b
    {0x38, 0x44, 0x44, 0x44, 0x20},  // c
    {0x38, 0x44, 0x44, 0x48, 0x7F},  // d
    {0x38, 0x54, 0x54, 0x54, 0x18},  // e
    {0x08, 0x7E, 0x09, 0x01, 0x02},  // f
    {0x08, 0x14, 0x54, 0x54, 0x3C},  // g
    {0x7F, 0x08, 0x04, 0x04, 0x78},  // h
    {0x00, 0x44, 0x7D, 0x40, 0x00},  // i
    {0x20, 0x40, 0x44, 0x3D, 0x00},  // j
    {0x00, 0x7F, 0x10, 0x28, 0x44},  // k
    {0x00, 0x41, 0x7F, 0x40, 0x00},  // l
    {0x7C, 0x04, 0x18, 0x04, 0x78},  // m
    {0x7C, 0x08, 0x04, 0x04, 0x78},  // n
    {0x38, 0x44, 0x44, 0x44, 0x38},  // o
    {0x7C, 0x14, 0x14, 0x14, 0x08},  // p
    {0x08, 0x14, 0x14, 0x18, 0x7C},  // q
    {0x7C, 0x08, 0x04, 0x04, 0x08},  // r
    {0x48, 0x54, 0x54, 0x54, 0x20},  // s
    {0x04, 0x3F, 0x44, 0x40, 0x20},  // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C},  // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C},  // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C},  // w
    {0x44, 0x28, 0x10, 0x28, 0x44},  // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C},  // y
    {0x44, 0x64, 0x54, 0x4C, 0x44},  // z
    {0x00, 0x08, 0x36, 0x41, 0x00},  // {
    {0x00, 0x00, 0x7F, 0x00, 0x00},  // |
    {0x00, 0x41, 0x36, 0x08, 0x00},  // }
    {0x08, 0x08, 0x2A, 0x1C, 0x08},  // ->
    {0x08, 0x1C, 0x2A, 0x08, 0x08}  // <-

};


const uint8_t Font5x7_180[][7] PROGMEM = {
    {0, 0, 0, 0, 0, 0, 0},
    {4, 0, 4, 4, 4, 4, 4},
    {0, 0, 0, 0, 10, 10, 10},
    {10, 10, 31, 10, 31, 10, 10},
    {4, 15, 20, 14, 5, 30, 4},
    {24, 25, 2, 4, 8, 19, 3},
    {22, 9, 21, 2, 5, 9, 6},
    {0, 0, 0, 0, 2, 4, 6},
    {8, 4, 2, 2, 2, 4, 8},
    {2, 4, 8, 8, 8, 4, 2},
    {0, 10, 4, 31, 4, 10, 0},
    {0, 4, 4, 31, 4, 4, 0},
    {2, 4, 6, 0, 0, 0, 0},
    {0, 0, 0, 31, 0, 0, 0},
    {6, 6, 0, 0, 0, 0, 0},
    {0, 1, 2, 4, 8, 16, 0},
    {14, 17, 19, 21, 25, 17, 14},
    {14, 4, 4, 4, 4, 6, 4},
    {31, 2, 4, 8, 16, 17, 14},
    {14, 17, 16, 8, 4, 8, 31},
    {8, 8, 31, 9, 10, 12, 8},
    {14, 17, 16, 16, 15, 1, 31},
    {14, 17, 17, 15, 1, 2, 12},
    {2, 2, 2, 4, 8, 16, 31},
    {14, 17, 17, 14, 17, 17, 14},
    {6, 8, 16, 30, 17, 17, 14},
    {0, 6, 6, 0, 6, 6, 0},
    {2, 4, 6, 0, 6, 6, 0},
    {16, 8, 4, 2, 4, 8, 16},
    {0, 0, 31, 0, 31, 0, 0},
    {1, 2, 4, 8, 4, 2, 1},
    {4, 0, 4, 8, 16, 17, 14},
    {14, 21, 21, 22, 16, 17, 14},
    {17, 17, 31, 17, 17, 17, 14},
    {15, 17, 17, 15, 17, 17, 15},
    {14, 17, 1, 1, 1, 17, 14},
    {7, 9, 17, 17, 17, 9, 7},
    {31, 1, 1, 15, 1, 1, 31},
    {1, 1, 1, 7, 1, 1, 31},
    {14, 17, 25, 1, 1, 17, 14},
    {17, 17, 17, 31, 17, 17, 17},
    {14, 4, 4, 4, 4, 4, 14},
    {6, 9, 8, 8, 8, 8, 28},
    {17, 9, 5, 3, 5, 9, 17},
    {31, 1, 1, 1, 1, 1, 1},
    {17, 17, 17, 17, 21, 27, 17},
    {17, 17, 25, 21, 19, 17, 17},
    {14, 17, 17, 17, 17, 17, 14},
    {1, 1, 1, 15, 17, 17, 15},
    {22, 9, 21, 17, 17, 17, 14},
    {17, 9, 5, 15, 17, 17, 15},
    {15, 16, 16, 14, 1, 1, 30},
    {4, 4, 4, 4, 4, 4, 31},
    {14, 17, 17, 17, 17, 17, 17},
    {4, 10, 17, 17, 17, 17, 17},
    {17, 27, 21, 21, 17, 17, 17},
    {17, 17, 10, 4, 10, 17, 17},
    {4, 4, 4, 4, 10, 17, 17},
    {31, 1, 2, 4, 8, 16, 31},
    {28, 4, 4, 4, 4, 4, 28},
    {0, 16, 8, 4, 2, 1, 0},
    {7, 4, 4, 4, 4, 4, 7},
    {0, 0, 0, 0, 17, 10, 4},
    {31, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 8, 4, 2},
    {30, 17, 30, 16, 14, 0, 0},
    {15, 17, 17, 19, 13, 1, 1},
    {14, 17, 1, 1, 14, 0, 0},
    {30, 17, 17, 25, 22, 16, 16},
    {14, 1, 31, 17, 14, 0, 0},
    {2, 2, 2, 7, 2, 18, 12},
    {12, 16, 30, 17, 30, 0, 0},
    {17, 17, 17, 19, 13, 1, 1},
    {14, 4, 4, 4, 6, 0, 4},
    {6, 9, 8, 8, 12, 0, 8},
    {18, 10, 6, 10, 18, 2, 2},
    {14, 4, 4, 4, 4, 4, 6},
    {17, 17, 21, 21, 11, 0, 0},
    {17, 17, 17, 19, 13, 0, 0},
    {14, 17, 17, 17, 14, 0, 0},
    {1, 1, 15, 17, 15, 0, 0},
    {16, 16, 30, 25, 22, 0, 0},
    {1, 1, 1, 19, 13, 0, 0},
    {15, 16, 14, 1, 14, 0, 0},
    {12, 18, 2, 2, 7, 2, 2},
    {22, 25, 17, 17, 17, 0, 0},
    {4, 10, 17, 17, 17, 0, 0},
    {10, 21, 21, 17, 17, 0, 0},
    {17, 10, 4, 10, 17, 0, 0},
    {14, 16, 30, 17, 17, 0, 0},
    {31, 2, 4, 8, 31, 0, 0},
    {8, 4, 4, 2, 4, 4, 8},
    {4, 4, 4, 4, 4, 4, 4},
    {2, 4, 4, 8, 4, 4, 2},
    {0, 4, 8, 31, 8, 4, 0},
    {0, 4, 2, 31, 2, 4, 0},
};

// ================== REGISTERS ==================
#define CONF_REG            0x00
#define PWM_REG             0x19
#define COLUMN_UPDATE_REG   0x0C
#define LIGHT_EFFECT_REG    0x0D

// ================== CONSTANTS ==================
#define PWM_CYCLE           127

#define LED1 0x60
#define LED2 0x61

#define CONFIG_DATA   0b00011000
#define LIGHT_CONFIG  0b00001110



// --- MM5450 input pins ---
const int PIN_CLK  = 2; // INT0
const int PIN_EN   = 3; // INT1
const int PIN_DATA = 4; // normal digital input

volatile uint8_t  bitIndex = 0;
volatile uint8_t  buffer[5];
volatile bool     receiving = false;
volatile bool     frameDone = false;


//--------------------------------------------------------------
// ENABLE edge handler (both rising + falling)
//--------------------------------------------------------------
void onEnableChange() {
    bool en = digitalRead(PIN_EN);   // HIGH = rising, LOW = falling

    if (!en) {
        // Falling edge → START of MM5450 frame
        receiving = true;
        frameDone = false;
        bitIndex = 0;

        for (int i = 0; i < 5; i++)
            buffer[i] = 0;

    } else {
        // Rising edge → END of MM5450 frame
        if (receiving) {
            receiving = false;

            if (bitIndex == 36) {
                frameDone = true;   // VALID frame
            }
            // else → ignore incomplete frame
        }
    }
}




void onClockRise() {
    if (!receiving) return;
    if (bitIndex >= 36) return;

    // Direct PORTD read: PIN_DATA = 4, masks bit 4
    uint8_t bit = (PIND & _BV(4)) ? 1 : 0;

    uint8_t byteIndex = bitIndex >> 3;     // faster than /8
    uint8_t bitPos    = 7 - (bitIndex & 7); // faster than %8

    buffer[byteIndex] |= (bit << bitPos);
    bitIndex++;
}


// =================================================
// I2C HELPERS
// =================================================
void writeByte(uint8_t addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void writeBlock(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  for (uint8_t i = 0; i < len; i++) {
    Wire.write(data[i]);
  }
  Wire.endTransmission();
}

// =================================================
// DRIVER INIT
// =================================================
void driverInit(uint8_t address) {
  writeByte(address, CONF_REG, CONFIG_DATA);
  writeByte(address, PWM_REG, PWM_CYCLE);
  writeByte(address, LIGHT_EFFECT_REG, LIGHT_CONFIG);
}

// =================================================
// PRINT 2 CHARACTERS (same logic as Python)
// =================================================
void printChars(uint8_t address, char c1, char c2, bool dp=false, bool pause = false, bool repeat = false, bool track_time = false) {
  uint8_t buffer[8]  = {0};
  uint8_t buffer2[8] = {0};

  uint8_t idx1 = c1 - 0x20;
  uint8_t idx2 = c2 - 0x20;

  // ----- First character (normal font)
  buffer[0] = dp ? 0b01000000 : 0x00;


  for (int i = 0; i < 5; i++) {
    buffer[i + 1] = pgm_read_byte(&Font5x7_full[idx1][i]);
  }

  // ----- Second character (rotated 180° font)
  for (int i = 0; i < 7; i++) {
    buffer2[i] = pgm_read_byte(&Font5x7_180[idx2][6 - i]) << 1;
  }

  if(track_time) {
    buffer[7] = buffer[7] | 0b00001000;
  } else {
    buffer[7] = buffer[7] | 0b00010000;
  }

  if (pause){
    buffer[7] = buffer[7] | 0b00000100;
  }

  if (repeat){
    buffer[7] = buffer[7] | 0b00000010;
  }

  writeBlock(address, 0x01, buffer2, 8);
  writeBlock(address, 0x0E, buffer, 8);
  writeByte(address, COLUMN_UPDATE_REG, 0x00);
}


char fontReverse(uint8_t v) {
  switch (v) {
    case 0b00111111: return '0';
    case 0b00000110: return '1';
    case 0b01011011: return '2';
    case 0b01001111: return '3';
    case 0b01100110: return '4';
    case 0b01101101: return '5';
    case 0b01111101: return '6';
    case 0b00000111: return '7';
    case 0b01111111: return '8';
    case 0b01101111: return '9';
    case 0b10000000: return '.';

    case 0b01110011: return 'P';
    case 0b00111000: return 'L';
    case 0b01110111: return 'A';
    case 0b01111100: return 'B';
    case 0b00111001: return 'C';
    case 0b01011110: return 'D';
    case 0b00011110: return 'J';
    case 0b00111101: return 'G';
    case 0b01101110: return 'Y';
    case 0b01111000: return 'T';
    case 0b01011001: return 'K';
    case 0b01011100: return 'O';
    case 0b00111110: return 'U';
    case 0b01111001: return 'E';
    case 0b01110001: return 'F';

    case 0b00000000: return ' ';
    case 0b01110110: return 'H';
    case 0b00010101: return 'M';
    case 0b01010100: return 'N';
    case 0b00110000: return 'I';
    case 0b01100111: return 'Q';
    case 0b01010000: return 'R';
    case 0b01000000: return '-';
  }
  return '?';  // unknown pattern
}

uint8_t reverse8(uint8_t n) {
    n = (n >> 4) | (n << 4);
    n = ((n & 0xCC) >> 2) | ((n & 0x33) << 2);
    n = ((n & 0xAA) >> 1) | ((n & 0x55) << 1);
    return n;
}


// =================================================
// SETUP
// =================================================
void setup() {
  Wire.begin();

  Serial.begin(115200);
  Serial.println("MM5450 Receiver Ready");

  pinMode(PIN_CLK,  INPUT);
  pinMode(PIN_EN,   INPUT);
  pinMode(PIN_DATA, INPUT);

  // ENABLE: detect RISING + FALLING
  attachInterrupt(digitalPinToInterrupt(PIN_EN), onEnableChange, CHANGE);

  // CLOCK: rising edge
  attachInterrupt(digitalPinToInterrupt(PIN_CLK), onClockRise, RISING);

  driverInit(LED1);
  driverInit(LED2);


}

// =================================================
// LOOP
// =================================================
void loop() {
      if (frameDone) {
        frameDone = false;

        uint8_t b1 = buffer[0];
        uint8_t b2 = buffer[1];
        uint8_t b3 = buffer[2];
        uint8_t b4 = buffer[3];
        uint8_t nibble = buffer[4] >> 4;

        // MM5450: first bit must be 1
        if (b1 & 0b10000000)
        {
            // Convert to LED driver layout
            // vbuffer[0] = reverse8(b1) >> 1;
            // vbuffer[1] = (reverse8(b2) >> 1) | ( b3 & 0b10000000);
            // vbuffer[2] = reverse8(b3) >> 1;
            // vbuffer[3] = reverse8(b4) >> 1;
            // vbuffer[4] = 0x00;
            // vbuffer[5] = 0x00;
            // vbuffer[6] = 0x00;
            // vbuffer[7] = 0x00;

            // // Send to LED driver
            // writeBlock(0x0E, vbuffer, 8);
            // writeReg(COLUMN_UPDATE_REG, 0x00);

            printChars(LED1, fontReverse(reverse8(b1) >> 1 ), fontReverse( reverse8(b2) >> 1 ), false, ( b2 & 0b10000000) == 0b10000000, nibble == 4, ( b4 & 0b10000000) == 0b10000000  );
            printChars(LED2, fontReverse(reverse8(b3) >> 1 ), fontReverse( reverse8(b4) >> 1 ), ( b3 & 0b10000000) == 0b10000000, false, false );

            Serial.println("Frame:"); 
            Serial.print("b1 = "); Serial.println( b1, BIN );
            Serial.print("b2 = "); Serial.println(b2, BIN ); 
            Serial.print("b3 = "); Serial.println( b3, BIN );
            Serial.print("b4 = "); Serial.println( b4, BIN);
            Serial.print("nibble = 0x"); Serial.println(nibble); 
            Serial.println("-----------");
            
            
        }
    }
}
