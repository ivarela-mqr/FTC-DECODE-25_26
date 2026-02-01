package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.ColorSensor;
@Autonomous
public class ColorSensorTest extends OpMode {
    ColorSensor colorSensor = new ColorSensor();
    ColorSensor.DetectedColors detectedColor;

    @Override
    public void init() {
        colorSensor.init(hardwareMap);

    }

    @Override
    public void loop() {
        detectedColor = colorSensor.getDetectedColor(telemetry);
        telemetry.addData("Color detected",detectedColor);

    }
}
