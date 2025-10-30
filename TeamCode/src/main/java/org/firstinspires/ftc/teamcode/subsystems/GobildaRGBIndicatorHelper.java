package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GobildaRGBIndicatorHelper extends SubsystemBase {
    /// Colour values for the Gobilda RGB Indicator Light
    public enum Colour {
        OFF(0.000),
        RED(0.3),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.500),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.000);

        public final double colourVal;

        Colour(double colourVal) {
            this.colourVal = colourVal;
        }

        public double getColourVal() {
            return colourVal;
        }
    }

    private Servo rgbIndicator;

    /**
     * Constructor for the Gobilda RGB Indicator Helper Class
     * @param hardwareMap gets the hardware map for using in initializing devices
     */
    public GobildaRGBIndicatorHelper(HardwareMap hardwareMap) {
        rgbIndicator = hardwareMap.get(Servo.class, "rgbIndicator");
        setColour(Colour.RED);
    }

    /**
     * Method to set the colour of the RGB light
     * @param colour the colour to set the light to
     */
    public void setColour(Colour colour) {
        rgbIndicator.setPosition(colour.getColourVal());
    }

    /**
     * Used to get the current colour for telemetry purposes
     * @return returns the closest matching Colour enum as a String
     */
    public String getCurrentColour() {
        double currentPos = rgbIndicator.getPosition();
        Colour closest = Colour.OFF;
        double smallestDiff = Double.MAX_VALUE;

        for (Colour c : Colour.values()) {
            double diff = Math.abs(c.getColourVal() - currentPos);
            if (diff < smallestDiff) {
                smallestDiff = diff;
                closest = c;
            }
        }

        return closest.name();
    }
}
