
const int PIN_DATA = 3;     // D3
const int PIN_CLK  = 2;     // D2
const int PIN_EN   = 4;     // D4

void sendBit(bool bitVal) {
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_DATA, bitVal ? HIGH : LOW);
  digitalWrite(PIN_CLK, HIGH);
}

void sendMM5450(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t nibble)
{

  digitalWrite(PIN_EN, LOW);

  sendByte(b1);
  sendByte(b2);
  sendByte(b3);
  sendByte(b4);

  // --- 3) Send low nibble (4 bits)
  for (int i = 3; i >= 0; i--) {
    sendBit((nibble >> i) & 1);
  }

  // DONE: return ENABLE high to latch data
  digitalWrite(PIN_EN, HIGH);
}

void sendByte(uint8_t value)
{
  for (int i = 7; i >= 0; i--) {
    sendBit((value >> i) & 1);
  }
}

void setup() {
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
  pinMode(PIN_EN, OUTPUT);

  digitalWrite(PIN_DATA, LOW);
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_EN, HIGH); // idle = disabled
}


byte digits_bin[10] = {
  0b01111110, // 0
  0b00110000, // 1
  0b01101101, // 2
  0b01111001, // 3
  0b00110011, // 4
  0b01011011, // 5
  0b01011111, // 6
  0b01110000, // 7
  0b01111111, // 8
  0b01111011  // 9
};

// byte hex_bin[6] = {
//   0b01110111, // A
//   0b00111110, // b
//   0b10011100, // C
//   0b01111010, // d
//   0b10011110, // E
//   0b10001110  // F
// };



void loop() {
  // example pattern
  sendMM5450(digits_bin[3] | 0b10000000, digits_bin[2], digits_bin[1], digits_bin[0], 0x00);

  delay(500);

  sendMM5450(0x00 | 0b10000000 , 0x00, 0x00, 0x00, 0x00);

  delay(500);
}