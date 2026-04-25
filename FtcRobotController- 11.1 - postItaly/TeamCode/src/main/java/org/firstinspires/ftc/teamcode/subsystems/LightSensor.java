package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp
public class LightSensor extends OpMode {
    public TouchSensor lightSensor;
    public TouchSensor lightSensor2;


    @Override
    public void init() {
        lightSensor = hardwareMap.get(TouchSensor.class,"lightSensor");
        //lightSensor2 = hardwareMap.get(TouchSensor.class,"lightSensor2");
    }

    @Override
    public void loop() {
         telemetry.addData("Hay bola",(lightSensor.isPressed() || lightSensor2.isPressed()));
    }
}
