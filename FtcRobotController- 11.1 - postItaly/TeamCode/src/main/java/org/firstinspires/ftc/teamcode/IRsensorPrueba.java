package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.LightSensor;

@TeleOp
public class IRsensorPrueba extends OpMode {
    LightSensor sensor;

    @Override
    public void init() {
        sensor.init();

    }



    @Override
    public void loop() {
        boolean detecting = sensor.lightSensor.isPressed();


        telemetry.addData("value",detecting);
        telemetry.update();
    }
}
