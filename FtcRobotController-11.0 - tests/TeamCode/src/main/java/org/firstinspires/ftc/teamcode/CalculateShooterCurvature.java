package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.security.Provider;
@TeleOp
public class CalculateShooterCurvature extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    Servo coverL, coverR;
    double distance = 0;
    double F = 0;
    double[] steps = {0.1, 0.001, 0.0001};
    DcMotorEx shooter1, shooter2;
    DcMotorEx  intake, transfer;
    double highVel = 1700;
    double lowVel = 1000;
    double currVel = highVel;
    int stepIndex = 0;
    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients coeficients = new PIDFCoefficients(1,0,0,15.4);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(8); // Blue alliance aprilTag
        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        coverL = hardwareMap.get(Servo.class,"coverL");
        coverR = hardwareMap.get(Servo.class,"coverR");
        coverL.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start(){
        limelight.start();
    }
    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            distance = getDistanceFromTargeta(llResult.getTa());
        }
        if(gamepad1.dpadLeftWasPressed())
            F += steps[stepIndex];
        if(gamepad1.dpadRightWasPressed())
            F -= steps[stepIndex];
        if(gamepad1.bWasPressed())
            stepIndex = (stepIndex + 1) % steps.length;
        coverL.setPosition(F);
        coverR.setPosition(F);
        telemetry.addData("Position",coverR.getPosition());
        telemetry.addData("Distance",distance);
        telemetry.update();
        intake.setPower(0.75);
        transfer.setPower(0.75);
        if(Math.abs(shooter1.getVelocity()-currVel)>100){
            shooter1.setPower(1);
            shooter2.setPower(1);
        }else {
            shooter1.setVelocity(currVel);
            shooter2.setVelocity(currVel);
        }
    }
    private double getDistanceFromTargeta(double ta){
        return 180.5062* Math.pow(ta,-0.5018798);
    }
}
