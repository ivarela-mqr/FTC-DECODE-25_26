package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
public class LightSensor {
    public TouchSensor lightSensor;
    public TouchSensor lightSensor2;


    LightSensor(HardwareMap hardwareMap, String name1, String name2){
        lightSensor = hardwareMap.get(TouchSensor.class,name1);
        lightSensor2 = hardwareMap.get(TouchSensor.class,name2);
    }

    public boolean isDetecting (){
        return (lightSensor.isPressed() || lightSensor2.isPressed());
    }

}
