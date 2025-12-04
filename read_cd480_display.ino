#include <Wire.h>

#define LED_DRIVER_ADDRESS 0x60

#define CONF_REG           0x00
#define PWM_REG            0x19
#define COLUMN_UPDATE_REG  0x0C
#define RESET_REG          0xFF
#define LIGHT_EFFECT_REG   0x0D

// CONFIGURE matrix mode, SSD, DM, A_EN, ADM
#define CONFIG_DATA        0b00001000

// SET intensity control settings
#define LIGHT_CONFIG       0b00001110

// Display buffer (8 bytes)
uint8_t vbuffer[8];


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



//--------------------------------------------------------------
// I2C helper functions
//--------------------------------------------------------------
void writeReg(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(LED_DRIVER_ADDRESS);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}


void writeBlock(uint8_t startReg, uint8_t *data, uint8_t len) {
    Wire.beginTransmission(LED_DRIVER_ADDRESS);
    Wire.write(startReg);
    for (uint8_t i = 0; i < len; i++)
        Wire.write(data[i]);
    Wire.endTransmission();
}



//--------------------------------------------------------------
// Bit-reverse (needed for MM5450 → your LED driver)
//--------------------------------------------------------------
uint8_t reverse8(uint8_t n) {
    n = (n >> 4) | (n << 4);
    n = ((n & 0xCC) >> 2) | ((n & 0x33) << 2);
    n = ((n & 0xAA) >> 1) | ((n & 0x55) << 1);
    return n;
}



//--------------------------------------------------------------
// SETUP
//--------------------------------------------------------------
void setup() {
    Wire.begin();
    delay(50);

    // Configure LED driver
    writeReg(CONF_REG, CONFIG_DATA);
    writeReg(PWM_REG, 64);
    writeReg(LIGHT_EFFECT_REG, LIGHT_CONFIG);

    Serial.begin(115200);
    Serial.println("MM5450 Receiver Ready");

    pinMode(PIN_CLK,  INPUT);
    pinMode(PIN_EN,   INPUT);
    pinMode(PIN_DATA, INPUT);

    // ENABLE: detect RISING + FALLING
    attachInterrupt(digitalPinToInterrupt(PIN_EN), onEnableChange, CHANGE);

    // CLOCK: rising edge
    attachInterrupt(digitalPinToInterrupt(PIN_CLK), onClockRise, RISING);
}



//--------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------
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
            vbuffer[0] = reverse8(b1) >> 1;
            vbuffer[1] = (reverse8(b2) >> 1) | ( b3 & 0b10000000);
            vbuffer[2] = reverse8(b3) >> 1;
            vbuffer[3] = reverse8(b4) >> 1;
            vbuffer[4] = 0x00;
            vbuffer[5] = 0x00;
            vbuffer[6] = 0x00;
            vbuffer[7] = 0x00;

            // Send to LED driver
            writeBlock(0x0E, vbuffer, 8);
            writeReg(COLUMN_UPDATE_REG, 0x00);

            // Serial.println("Frame:"); 
            // Serial.print("b1 = "); Serial.println(b1, BIN);
            // Serial.print("b2 = "); Serial.println(b2, BIN); 
            // Serial.print("b3 = "); Serial.println(b3, BIN);
            // Serial.print("b4 = "); Serial.println(b4, BIN);
            // Serial.print("nibble = 0x"); Serial.println(nibble, HEX); 
            // Serial.println("-----------");
            
            
        }
    }
}
