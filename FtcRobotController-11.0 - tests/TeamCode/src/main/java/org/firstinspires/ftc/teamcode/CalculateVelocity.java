package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.LimeLight;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;

@TeleOp
public class CalculateVelocity extends OpMode {
    Shooter shooter;
    LimeLight limeLight;
    double vel = 1200,pos = 0;
    double stepVel = 50,stepCover = 0.05;
    public DcMotorEx transfer;
    DcMotor intake;
    IMU imu;
    @Override
    public void init() {
        shooter = new Shooter(hardwareMap,vel);
        limeLight = new LimeLight(hardwareMap);

        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        shooter.TeleOp(vel,pos);
        if(gamepad1.dpadUpWasPressed())
            vel += stepVel;
        if(gamepad1.dpadDownWasPressed())
            vel -= stepVel;
        if(gamepad1.dpadLeftWasPressed())
            pos -= stepCover;
        if(gamepad1.dpadRightWasPressed())
            pos += stepCover;

        intake.setPower(1);
        transfer.setPower(1);
        telemetry.addData("Velocity",vel);
        telemetry.addData("Target velocity", Math.max(shooter.shooter1.getVelocity(),shooter.shooter0.getVelocity()));
        telemetry.addData("Pos",pos);
        telemetry.addData("Distance",
                limeLight.getGoalAprilTagData(imu.getRobotYawPitchRollAngles().getYaw())[1]);
    }
}
