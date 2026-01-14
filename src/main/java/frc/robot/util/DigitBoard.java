package frc.robot.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;

/**
 * Subsystem to control communication with a Rev DigitBoard. Based on sample
 * code found <a href="https://github.com/vampjaz/REVDigitBoard/blob/master/REVDigitBoard.java">here
 * on github</a>
 */
public class DigitBoard {

    private static final byte [] OSC = { (byte) 0x21 };
    private static final byte [] BLINK = { (byte) 0x81 };
    private static final byte [] BRIGHT = { (byte) 0xEf };

    // Adapted partially from Team 1493, the rest was done using regular expressions!
    private static final byte[][] CHARS =
            {
                    {(byte)0b00000000, (byte)0b00000000}, //
                    {(byte)0b00000110, (byte)0b00000000}, // !
                    {(byte)0b00100000, (byte)0b00000010}, // "
                    {(byte)0b11001110, (byte)0b00010010}, // #
                    {(byte)0b11101101, (byte)0b00010010}, // $
                    {(byte)0b00100100, (byte)0b00100100}, // %
                    {(byte)0b01011101, (byte)0b00001011}, // &
                    {(byte)0b00000000, (byte)0b00000100}, // '
                    {(byte)0b00000000, (byte)0b00001100}, // (
                    {(byte)0b00000000, (byte)0b00100001}, // )
                    {(byte)0b11000000, (byte)0b00111111}, // *
                    {(byte)0b11000000, (byte)0b00010010}, // +
                    {(byte)0b00000000, (byte)0b00100000}, // ,
                    {(byte)0b11000000, (byte)0b00000000}, // -
                    {(byte)0b00000000, (byte)0b00000000}, // .
                    {(byte)0b00000000, (byte)0b00100100}, // /
                    {(byte)0b00111111, (byte)0b00100100}, // 0
                    {(byte)0b00000110, (byte)0b00000000}, // 1
                    {(byte)0b11011011, (byte)0b00000000}, // 2
                    {(byte)0b10001111, (byte)0b00000000}, // 3
                    {(byte)0b11100110, (byte)0b00000000}, // 4
                    {(byte)0b01101001, (byte)0b00001000}, // 5
                    {(byte)0b11111101, (byte)0b00000000}, // 6
                    {(byte)0b00000111, (byte)0b00000000}, // 7
                    {(byte)0b11111111, (byte)0b00000000}, // 8
                    {(byte)0b11101111, (byte)0b00000000}, // 9
                    {(byte)0b00000000, (byte)0b00010010}, // :
                    {(byte)0b00000000, (byte)0b00100010}, // ;
                    {(byte)0b00000000, (byte)0b00001100}, // <
                    {(byte)0b11001000, (byte)0b00000000}, // =
                    {(byte)0b00000000, (byte)0b00100001}, // >
                    {(byte)0b10000011, (byte)0b00010000}, // ?
                    {(byte)0b10111011, (byte)0b00000010}, // @
                    {(byte)0b11110111, (byte)0b00000000}, // A
                    {(byte)0b10001111, (byte)0b00010010}, // B
                    {(byte)0b00111001, (byte)0b00000000}, // C
                    {(byte)0b00001111, (byte)0b00010010}, // D
                    {(byte)0b11111001, (byte)0b00000000}, // E
                    {(byte)0b01110001, (byte)0b00000000}, // F
                    {(byte)0b10111101, (byte)0b00000000}, // G
                    {(byte)0b11110110, (byte)0b00000000}, // H
                    {(byte)0b00000000, (byte)0b00010010}, // I
                    {(byte)0b00011110, (byte)0b00000000}, // J
                    {(byte)0b01110000, (byte)0b00001100}, // K
                    {(byte)0b00111000, (byte)0b00000000}, // L
                    {(byte)0b00110110, (byte)0b00000101}, // M
                    {(byte)0b00110110, (byte)0b00001001}, // N
                    {(byte)0b00111111, (byte)0b00000000}, // O
                    {(byte)0b11110011, (byte)0b00000000}, // P
                    {(byte)0b00111111, (byte)0b00001000}, // Q
                    {(byte)0b11110011, (byte)0b00001000}, // R
                    {(byte)0b11101101, (byte)0b00000000}, // S
                    {(byte)0b00000001, (byte)0b00010010}, // T
                    {(byte)0b00111110, (byte)0b00000000}, // U
                    {(byte)0b00110000, (byte)0b00100100}, // V
                    {(byte)0b00110110, (byte)0b00101000}, // W
                    {(byte)0b00000000, (byte)0b00101101}, // X
                    {(byte)0b00000000, (byte)0b00010101}, // Y
                    {(byte)0b00001001, (byte)0b00100100}, // Z
                    {(byte)0b00111001, (byte)0b00000000}, // [
                    {(byte)0b00000000, (byte)0b00001001}, // \
                    {(byte)0b00001111, (byte)0b00000000}, // ]
                    {(byte)0b00000011, (byte)0b00100100}, // ^
                    {(byte)0b00001000, (byte)0b00000000}, // _
                    {(byte)0b00000000, (byte)0b00000001}, // `
                    {(byte)0b01011000, (byte)0b00010000}, // a
                    {(byte)0b01111000, (byte)0b00001000}, // b
                    {(byte)0b11011000, (byte)0b00000000}, // c
                    {(byte)0b10001110, (byte)0b00100000}, // d
                    {(byte)0b01011000, (byte)0b00100000}, // e
                    {(byte)0b01110001, (byte)0b00000000}, // f
                    {(byte)0b10001110, (byte)0b00000100}, // g
                    {(byte)0b01110000, (byte)0b00010000}, // h
                    {(byte)0b00000000, (byte)0b00010000}, // i
                    {(byte)0b00001110, (byte)0b00000000}, // j
                    {(byte)0b00000000, (byte)0b00011110}, // k
                    {(byte)0b00110000, (byte)0b00000000}, // l
                    {(byte)0b11010100, (byte)0b00010000}, // m
                    {(byte)0b01010000, (byte)0b00010000}, // n
                    {(byte)0b11011100, (byte)0b00000000}, // o
                    {(byte)0b01110000, (byte)0b00000001}, // p
                    {(byte)0b10000110, (byte)0b00000100}, // q
                    {(byte)0b01010000, (byte)0b00000000}, // r
                    {(byte)0b10001000, (byte)0b00001000}, // s
                    {(byte)0b01111000, (byte)0b00000000}, // t
                    {(byte)0b00011100, (byte)0b00000000}, // u
                    {(byte)0b00000100, (byte)0b00001000}, // v
                    {(byte)0b00010100, (byte)0b00101000}, // w
                    {(byte)0b11000000, (byte)0b00101000}, // x
                    {(byte)0b00001100, (byte)0b00001000}, // y
                    {(byte)0b01001000, (byte)0b00100000}, // z
                    {(byte)0b01001001, (byte)0b00100001}, // {
                    {(byte)0b00000000, (byte)0b00010010}, // |
                    {(byte)0b10001001, (byte)0b00001100}, // }
                    {(byte)0b00100000, (byte)0b00000101}, // ~
                    {(byte)0b11111111, (byte)0b00111111}  // DEL
            };

    private final I2C i2c;
    private final DigitalInput buttonA;
    private final DigitalInput buttonB;
    private final AnalogInput pot;
    private final byte [] buffer;
    private boolean aWasPressed;
    private boolean bWasPressed;

    /**
     * Creates a {@link DigitBoard}
     */
    public DigitBoard() {
        this.buffer = new byte[10];
        this.i2c = new I2C(I2C.Port.kMXP, 0x70);
        this.buttonA = new DigitalInput(19);
        this.buttonB = new DigitalInput(20);
        this.pot = new AnalogInput(3);
        this.pot.setAverageBits(10);
    }

    /**
     * @return true if button A is currently being pressed
     */
    public boolean isButtonAPressed() {
        return !buttonA.get();
    }

    /**
     * @return true if button A was pressed the last time this was called,
     * and is no longer being pressed now (this is just like the logic for
     * {@link edu.wpi.first.wpilibj.XboxController#getRawButtonReleased(int)}
     */
    public boolean wasButtonAReleased() {
        boolean isPressed = isButtonAPressed();
        boolean wasReleased = isPressed && !aWasPressed;
        aWasPressed = isPressed;
        return wasReleased;
    }

    /**
     * @return true if button B is currently being pressed
     */
    public boolean isButtonBPressed() {
        return !buttonB.get();
    }

    /**
     * @return true if button B was pressed the last time this was called,
     * and is no longer being pressed now (this is just like the logic for
     * {@link edu.wpi.first.wpilibj.XboxController#getRawButtonReleased(int)}
     */
    public boolean wasButtonBReleased() {
        boolean isPressed = isButtonBPressed();
        boolean wasReleased = isPressed && !bWasPressed;
        bWasPressed = isPressed;
        return wasReleased;
    }

    /**
     * @return the value of the potentiometer
     */
    public int getValue() {

        // we found that the actual value being returned is in a weird
        // range that seemed to vary from week to week. use at your own
        // peril

        return pot.getAverageValue();
    }

    /**
     * Writes the first 4 characters of the supplied string to the display
     * (the string will be right-padded with spaces if it's not long enough)
     *
     * @param text the text to display
     */
    public void display(String text) {
        writeToBuffer(text+"      ");
        i2c.writeBulk(OSC);
        i2c.writeBulk(BRIGHT);
        i2c.writeBulk(BLINK);
        i2c.writeBulk(buffer);
    }

    /*
     * Logic for writing to the buffer
     */
    private void writeToBuffer(String output)
    {
        buffer[0] = (byte)(0b0000111100001111);

        int offset = 0;

        for(int i = 0; i < 4; i++)
        {
            char letter = output.charAt(i + offset);

            while(/*letter < 32 ||*/ letter == '.')
            {
                // this code was copied verbatim and included this error
                // noinspection ConstantConditions
                if (letter == '.')
                {
                    if (i != 0) {
                        buffer[(4 - i) * 2 + 3] |= (byte) 0b01000000;
                    }
                }

                offset++;
                letter = output.charAt(i + offset);
            }
            buffer[(3-i)*2+2] = CHARS[letter-32][0];
            buffer[(3-i)*2+3] = CHARS[letter-32][1];
        }
    }
}
