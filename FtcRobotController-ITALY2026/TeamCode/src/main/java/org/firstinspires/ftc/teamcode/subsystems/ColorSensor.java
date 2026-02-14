package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ColorSensor {
    NormalizedColorSensor colorSensor;
    NormalizedColorSensor colorSensor2;
    public enum DetectedColors {
        PURPLE,
        GREEN,
        UNKNOWN
    }
    public void init(HardwareMap map){
        colorSensor = map.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor2 = map.get(NormalizedColorSensor.class, "colorSensor2");
        colorSensor.setGain(6);
        colorSensor2.setGain(6);
    }
    public DetectedColors getDetectedColor(){
        //RGB + alpha -> brightness
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        NormalizedRGBA colors2 = colorSensor2.getNormalizedColors();
        float normRed, normGreen, normBlue;
        float normRed2, normGreen2, normBlue2;
        normRed = colors.red / colors.alpha;
        normRed2 = colors2.red / colors2.alpha;
        normGreen = colors.green / colors.alpha;
        normGreen2 = colors2.green / colors2.alpha;
        normBlue = colors.blue / colors.alpha;
        normBlue2 = colors2.blue / colors2.alpha;
        /*telemetry.addData("red", normRed);
        telemetry.addData("red", normRed2);
        telemetry.addData("green", normGreen);
        telemetry.addData("green",normGreen2);
        telemetry.addData("blue", normBlue);
        telemetry.addData("blue", normBlue2);*/

        if (isGreen(normRed,normRed2,normGreen,normGreen2,normBlue,normBlue2)){
            return DetectedColors.GREEN;
        }
        else if (isPurple(normRed,normRed2,normGreen,normGreen2,normBlue,normBlue2)) {
            return DetectedColors.PURPLE;
        }
        else {
            return DetectedColors.UNKNOWN;
        }
    }
    public boolean isGreen(float r1, float r2, float g1, float g2, float b1, float b2) {

        boolean sensor1IsHollow =
                r1 > Constants.HOLLOW_GREEN_R
                        && g1 > Constants.HOLLOW_GREEN_G
                        && b1 > Constants.HOLLOW_GREEN_B;

        boolean sensor2IsHollow =
                r2 > Constants.HOLLOW_GREEN_R
                        && g2 > Constants.HOLLOW_GREEN_G
                        && b2 > Constants.HOLLOW_GREEN_B;

        boolean sensor1IsPlain =
                r1 > Constants.PLAIN_GREEN_R
                        && g1 > Constants.PLAIN_GREEN_G
                        && b1 > Constants.PLAIN_GREEN_B;

        boolean sensor2IsPlain =
                r2 > Constants.PLAIN_GREEN_R
                        && g2 > Constants.PLAIN_GREEN_G
                        && b2 > Constants.PLAIN_GREEN_B;

        return (sensor1IsPlain || sensor2IsPlain);
    }
    public boolean isPurple(float r1, float r2, float g1, float g2, float b1, float b2){
        boolean sensor1IsHollow =
                r1 > Constants.HOLLOW_PURPLE_R
                        && g1 > Constants.HOLLOW_PURPLE_G
                        && b1 > Constants.HOLLOW_PURPLE_B;

        boolean sensor2IsHollow =
                r2 > Constants.HOLLOW_PURPLE_R &&
                        g2 > Constants.HOLLOW_PURPLE_G &&
                        b2 > Constants.HOLLOW_PURPLE_B;

        boolean sensor1IsPlain =
                r1 > Constants.PLAIN_PURPLE_R &&
                        g1 > Constants.PLAIN_PURPLE_G &&
                        b1 > Constants.PLAIN_PURPLE_B;

        boolean sensor2IsPlain =
                r2 > Constants.PLAIN_PURPLE_R &&
                        g2 > Constants.PLAIN_PURPLE_G &&
                        b2 > Constants.PLAIN_PURPLE_B;

        return  (sensor1IsPlain || sensor2IsPlain);
    }
}
