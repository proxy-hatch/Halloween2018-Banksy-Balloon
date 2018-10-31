const uint16_t myImageWidth = 16;
const uint16_t myImageHeight = 16;
const uint8_t PROGMEM myImage[] = {  // (16 x 16) GRB in Hexadecimal
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0x00, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0xab, 0xab, 0xab, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0xaf, 0xad, 0xab, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0xaf, 0xad, 0xab, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0xaf, 0xad, 0xab, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0xaf, 0xad, 0xab, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0xaf, 0xad, 0xab, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0xab, 0xab, 0xab, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0xab, 0xab, 0xab, 0xaf, 0xad, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xaf, 0xad, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0xab, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xad, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0xab, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };