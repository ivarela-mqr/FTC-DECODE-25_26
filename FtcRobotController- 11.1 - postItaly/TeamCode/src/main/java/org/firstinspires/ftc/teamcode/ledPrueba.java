package org.firstinspires.ftc.teamcode;


import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;

@TeleOp
public class ledPrueba extends OpMode {
    Servo LED;
    double x = 0.25;

    @Override
    public void init() {
        LED = hardwareMap.get(Servo.class, "led");


    }



    @Override
    public void loop() {
        if(x<0.75){
            x = x + 0.0001;
        }else{
            x = 0.25;
        }
        LED.setPosition(x);
        telemetry.addData("value",x);
        telemetry.update();
    }
}
