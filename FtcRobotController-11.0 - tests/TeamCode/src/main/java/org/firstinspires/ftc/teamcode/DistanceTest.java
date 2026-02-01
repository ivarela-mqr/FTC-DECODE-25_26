package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.TestDistanceSensor;

@Autonomous
public class DistanceTest extends OpMode {
    TestDistanceSensor sensor = new TestDistanceSensor();
    @Override
    public void init() {
        sensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Distance in cm",sensor.getDistance());
    }
}
