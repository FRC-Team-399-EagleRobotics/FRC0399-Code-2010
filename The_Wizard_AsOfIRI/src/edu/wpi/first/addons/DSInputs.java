package edu.wpi.first.addons;

/**
 * This class handles most interactions with the DriverStation
 *      -Dashboard
 *      -LCD
 *      -Switchboard
 *      -I/O Board
 *
 * @author Jeremy Germita
 */

import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;

public class DSInputs {

    DriverStationEnhancedIO enhancedIO = 
            DriverStation.getInstance().getEnhancedIO();
    DriverStationLCD LCD =
            DriverStationLCD.getInstance();

    DSInputs() {

    }
    
    /*
     * Return boolean true if the desired switch is toggled on
     */
    public boolean getSwitch(int toggle) {  //User Defines Which Switch(1-4)

        toggle += 8;                        //Defined switch number is added to
                                            //8 so the board will recognise it
        
        if (toggle < 9 || toggle > 12) {    //If the number is under 1, or over
                                            //4, 
            return false;                   //Return FALSE
        }
        try {
            boolean switchOut = false;      
            if(enhancedIO.getDigital(toggle)) {
                switchOut = true;
            } else {
                switchOut = false;
            }
            return switchOut;
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
                System.out.println(ex.getMessage());
                return false;
        }

        

    }
    public double getRotary() {

        try {
            double anin = enhancedIO.getAnalogIn(2);
            return anin;
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
                System.out.println(ex.getMessage());
                return -1.0;
        }

    }

    public void setLED(int ledSet, boolean value) {
        try {
            enhancedIO.setLED(ledSet, value);
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
                System.out.println(ex.getMessage());

        }
    }


    public int fieldSelect() {
        int field = 0;

        if(getRotary() >= 1.0 && getRotary() <= 1.9) {
            field = 1;
        } else if(getRotary() >= 2.0 && getRotary() <= 2.9) {
            field = 2;
        } else if(getRotary() >= 3.0) {
            field = 3;
        } else {
            field = 0;
        }

        return field;
    }

    public int getSwitches() {              //Gets switches toggled,
                                            //returns a value, based on binary
    try {
        int sw1;
        int sw2;
        int sw3;
        int sw4;
        int autonMode = 0;

        if(enhancedIO.getDigital(9)) {
                sw1 = 1;
        } else {
                sw1 = 0;
        }
        if(enhancedIO.getDigital(10)) {
                sw2 = 2;
        } else {
                sw2 = 0;
        }
        if(enhancedIO.getDigital(11)) {
                sw3 = 4;
        } else {
                sw3 = 0;
        }
        if(enhancedIO.getDigital(12)) {
                sw4 = 8;
        } else {
                sw4 = 0;
        }

        autonMode = (sw1 + sw2 + sw3 + sw4);       //converts the binary of the
                                                   //Switches into decimal
        return autonMode;

        /*
         * Switch Example:
         * Switches on (binary)     Returned Value
         * 1 0 0 0                  1
         * 0 1 0 0                  2
         * 1 1 0 0                  3
         * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
         * 1 1 1 1                  15
         */

    } catch(DriverStationEnhancedIO.EnhancedIOException ex) {
        return -1;
    }

    }
        public void lcdPrint(int sel) {
		
        if(sel == 1) {

            int field = fieldSelect();

            if(field == 1) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, ">>>>>>>>>>>>>>>>>>>>>");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "^    NEAR  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "^    NEAR  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "<<<<<<<<<<<<<<<<<<<<<");
            } else if(field == 2) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, ">>>>>>>>>>>>>>>>>>>>>");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "^     MID  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "^     MID  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "<<<<<<<<<<<<<<<<<<<<<");
            } else if(field == 3) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, ">>>>>>>>>>>>>>>>>>>>>");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "^     FAR  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "^     FAR  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "<<<<<<<<<<<<<<<<<<<<<");
            } else {
                LCD.println(DriverStationLCD.Line.kUser2, 1, "!!!!!!!!!!!!!!!!!!!!!");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "!        ERROR      !");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "!CHECK KNOB/IO BOARD!");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "!!!!!!!!!!!!!!!!!!!!!");
            }
			
        }
        else if(sel == 2) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, "                     ");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "                     ");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "                     ");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "                     ");

        }

        LCD.updateLCD();

    }
}
