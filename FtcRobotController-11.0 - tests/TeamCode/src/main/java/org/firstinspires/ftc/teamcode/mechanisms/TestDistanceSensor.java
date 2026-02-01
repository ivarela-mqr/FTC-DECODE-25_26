package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TestDistanceSensor {
    DistanceSensor sensor;

    public void init(HardwareMap map){
        sensor = map.get(DistanceSensor.class,"distanceSensor");
    }

    public double getDistance(){
        return sensor.getDistance(DistanceUnit.CM);
    }
}
