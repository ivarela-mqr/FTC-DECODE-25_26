package org.firstinspires.ftc.teamcode;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class pruebaTorreta extends OpMode {
    Servo rotorR;
    Servo rotorL;

    @Override
    public void init() {

    }

    @Override
    public void start() {
        gamepad1.rumble(20);
        gamepad2.rumble(20);
    }


    @Override
    public void loop() {


        if (gamepad1.dpad_up) {
            rotorL.setPosition(0.5);
            rotorR.setPosition(0.5);
        } else if (gamepad1.dpad_left) {
            rotorL.setPosition(0);
            rotorR.setPosition(0);
        } else if (gamepad1.dpad_right) {
            rotorL.setPosition(1);
            rotorR.setPosition(1);
        }
        telemetry.update();

    }
}