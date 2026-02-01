package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    DcMotorEx shooter0, shooter1;
    Servo rotorL, rotorR, cover;

    LimeLight limeLight;


    Shooter (HardwareMap hardwareMap, Constants.Alliance alliance){
        shooter0 = hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        rotorL = hardwareMap.get(Servo.class,"rotorL");
        rotorR = hardwareMap.get(Servo.class,"rotorR");
        cover = hardwareMap.get(Servo.class,"cover");

        limeLight = new LimeLight(hardwareMap, alliance);

    }

    public void aimShooterWithLimeLight(Telemetry telemetry,double yawAngle){
        double shooterOffset = limeLight.getGoalAprilTagData(telemetry, yawAngle)[0];

    }

    public void aimShooterWithOdometry(){}
    public void TeleOp(Gamepad gamepad, Telemetry telemetry){

    }

}
