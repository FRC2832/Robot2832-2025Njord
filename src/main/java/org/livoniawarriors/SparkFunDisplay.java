// this is an updated version of https://github.com/vampjaz/REVDigitBoard

package org.livoniawarriors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.Timer;
import java.util.*;

public class SparkFunDisplay {
  /*
   * DOCUMENTATION::
   *
   * REVDigitBoard() : constructor
   * void display(String str) : displays the first four characters of the string (only alpha (converted to uppercase), numbers, and spaces)
   * void display(double batt) : displays a decimal number (like battery voltage) in the form of 12.34 (ten-one-decimal-tenth-hundredth)
   * void clear() : clears the display
   * boolean getButtonA() : button A on the board
   * boolean getButtonB() : button B on the board
   * double getPot() : potentiometer value
   */

  I2C i2c;

  HashMap<Character, Integer> charmap;

  public SparkFunDisplay() {
    i2c = new I2C(Port.kMXP, 0x70);

    byte[] osc = new byte[1];
    byte[] blink = new byte[1];
    byte[] bright = new byte[1];
    osc[0] = (byte) 0x21;
    blink[0] = (byte) 0x81;
    bright[0] = (byte) 0xEF;

    i2c.writeBulk(osc);
    Timer.delay(.01);
    i2c.writeBulk(bright);
    Timer.delay(.01);
    i2c.writeBulk(blink);
    Timer.delay(.01);

    charmap = new HashMap<Character, Integer>();
    charmap.put(' ', 0b00000000000000);
    charmap.put('!', 0b00001000001000);
    charmap.put('"', 0b00001000000010);
    charmap.put('#', 0b01001101001110);
    charmap.put('$', 0b01001101101101);
    charmap.put('%', 0b10010000100100);
    charmap.put('&', 0b00110011011001);
    charmap.put('\'', 0b00001000000000);
    charmap.put('(', 0b00000000111001);
    charmap.put(')', 0b00000000001111);
    charmap.put('*', 0b11111010000000);
    charmap.put('+', 0b01001101000000);
    charmap.put(',', 0b10000000000000);
    charmap.put('-', 0b00000101000000);
    charmap.put('.', 0b00000000000000);
    charmap.put('/', 0b10010000000000);
    charmap.put('0', 0b00000000111111);
    charmap.put('1', 0b00010000000110);
    charmap.put('2', 0b00000101011011);
    charmap.put('3', 0b00000101001111);
    charmap.put('4', 0b00000101100110);
    charmap.put('5', 0b00000101101101);
    charmap.put('6', 0b00000101111101);
    charmap.put('7', 0b01010000000001);
    charmap.put('8', 0b00000101111111);
    charmap.put('9', 0b00000101100111);
    charmap.put(':', 0b00000000000000);
    charmap.put(';', 0b10001000000000);
    charmap.put('<', 0b00110000000000);
    charmap.put('=', 0b00000101001000);
    charmap.put('>', 0b01000010000000);
    charmap.put('?', 0b01000100000011);
    charmap.put('@', 0b00001100111011);
    charmap.put('A', 0b00000101110111);
    charmap.put('B', 0b01001100001111);
    charmap.put('C', 0b00000000111001);
    charmap.put('D', 0b01001000001111);
    charmap.put('E', 0b00000101111001);
    charmap.put('F', 0b00000101110001);
    charmap.put('G', 0b00000100111101);
    charmap.put('H', 0b00000101110110);
    charmap.put('I', 0b01001000001001);
    charmap.put('J', 0b00000000011110);
    charmap.put('K', 0b00110001110000);
    charmap.put('L', 0b00000000111000);
    charmap.put('M', 0b00010010110110);
    charmap.put('N', 0b00100010110110);
    charmap.put('O', 0b00000000111111);
    charmap.put('P', 0b00000101110011);
    charmap.put('Q', 0b00100000111111);
    charmap.put('R', 0b00100101110011);
    charmap.put('S', 0b00000110001101);
    charmap.put('T', 0b01001000000001);
    charmap.put('U', 0b00000000111110);
    charmap.put('V', 0b10010000110000);
    charmap.put('W', 0b10100000110110);
    charmap.put('X', 0b10110010000000);
    charmap.put('Y', 0b01010010000000);
    charmap.put('Z', 0b10010000001001);
    charmap.put('[', 0b00000000111001);
    charmap.put('\\', 0b00100010000000);
    charmap.put(']', 0b00000000001111);
    charmap.put('^', 0b10100000000000);
    charmap.put('_', 0b00000000001000);
    charmap.put('`', 0b00000010000000);
    charmap.put('a', 0b00000101011111);
    charmap.put('b', 0b00100001111000);
    charmap.put('c', 0b00000101011000);
    charmap.put('d', 0b10000100001110);
    charmap.put('e', 0b00000001111001);
    charmap.put('f', 0b00000001110001);
    charmap.put('g', 0b00000110001111);
    charmap.put('h', 0b00000101110100);
    charmap.put('i', 0b01000000000000);
    charmap.put('j', 0b00000000001110);
    charmap.put('k', 0b01111000000000);
    charmap.put('l', 0b01001000000000);
    charmap.put('m', 0b01000101010100);
    charmap.put('n', 0b00100001010000);
    charmap.put('o', 0b00000101011100);
    charmap.put('p', 0b00010001110001);
    charmap.put('q', 0b00100101100011);
    charmap.put('r', 0b00000001010000);
    charmap.put('s', 0b00000110001101);
    charmap.put('t', 0b00000001111000);
    charmap.put('u', 0b00000000011100);
    charmap.put('v', 0b10000000010000);
    charmap.put('w', 0b10100000010100);
    charmap.put('x', 0b10110010000000);
    charmap.put('y', 0b00001100001110);
    charmap.put('z', 0b10010000001001);
    charmap.put('{', 0b10000011001001);
    charmap.put('|', 0b01001000000000);
    charmap.put('}', 0b00110100001001);
    charmap.put('~', 0b00000101010010);
  }

  public void display(String str) { // only displays first 4 chars
    char buff;

    // Clear the displayRAM array
    for (int i = 0; i < 16; i++) displayRAM[i] = 0;

    int stringIndex = 0;
    digitPosition = 0;

    while (stringIndex < str.length() && digitPosition < 4) {
      buff = str.charAt(stringIndex);
      // For special characters like '.' or ':', do not increment the digitPosition
      if (buff == '.') printChar('.', (byte) 0);
      else if (buff == ':') printChar(':', (byte) 0);
      else {
        printChar(buff, digitPosition);
        digitPosition++;
      }
      stringIndex++;
    }

    // send the array to the board
    byte[] buffer = new byte[displayRAM.length + 1];

    // writing the display info starts at address 0
    buffer[0] = 0;
    for (int i = 0; i < displayRAM.length; i++) {
      buffer[i + 1] = displayRAM[i];
    }
    i2c.writeBulk(buffer);
  }

  byte digitPosition = 0;
  byte[] displayRAM = new byte[16];

  void printChar(char displayChar, byte digit) {
    // moved alphanumeric_segs array to PROGMEM
    int characterPosition = 65532;

    // space
    if (displayChar == ' ') characterPosition = 0;
    // Printable Symbols -- Between first character ! and last character ~
    else if (displayChar >= '!' && displayChar <= '~') {
      characterPosition = displayChar - '!' + 1;
    }

    byte dispNum = (byte) (digitPosition / 4);

    // Take care of special characters by turning correct segment on
    // if (characterPosition == 14) // '.'
    // 	decimalOnSingle(dispNum+1, false);
    // if (characterPosition == 26) // ':'
    // 	colonOnSingle(dispNum+1, false);
    if (characterPosition == 65532) // unknown character
    characterPosition = 95;

    int segmentsToTurnOn = charmap.get((char) displayChar);

    illuminateChar(segmentsToTurnOn, digit);
  }

  void illuminateChar(int segmentsToTurnOn, byte digit) {
    for (int i = 0; i < 14; i++) // There are 14 segments on this display
    {
      // 0b00100101110011
      if (((segmentsToTurnOn >> i) & 0b1) == 1)
        illuminateSegment((char) ('A' + i), digit); // Convert the segment number to a letter
    }
  }

  void illuminateSegment(char segment, byte digit) {
    byte com;
    byte row;

    com = (byte) (segment - 'A'); // Convert the segment letter back to a number

    if (com > 6) com -= 7;
    if (segment == 'I') com = 0;
    if (segment == 'H') com = 1;

    row =
        (byte)
            (digit % 4); // Convert digit (1 to 16) back to a relative position on a given digit on
    // display
    if (segment > 'G') row += 4;

    byte offset = (byte) (digit / 4 * 16);
    byte adr = (byte) (com * 2 + offset);

    // Determine the address
    if (row > 7) adr++;

    // Determine the data bit
    if (row > 7) row -= 8;
    byte dat = (byte) (1 << row);

    displayRAM[adr] = (byte) (displayRAM[adr] | dat);
  }

  public void display(double batt) { // optimized for battery voltage, needs a double like 12.34
    int[] charz = {36, 36, 36, 36};
    // idk how to decimal point
    int ten = (int) (batt / 10);
    int one = (int) (batt % 10);
    int tenth = (int) ((batt * 10) % 10);
    int hundredth = (int) ((batt * 100) % 10);

    if (ten != 0) charz[0] = ten;
    charz[1] = one;
    charz[2] = tenth;
    charz[3] = hundredth;

    // this._display(charz);
  }

  public void clear() {
    int[] charz = {36, 36, 36, 36}; // whyy java
    // this._display(charz);
  }

  ////// not supposed to be publicly used..

  String repeat(char c, int n) {
    char[] arr = new char[n];
    Arrays.fill(arr, c);
    return new String(arr);
  }
}
