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
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

import java.security.Provider;
@TeleOp
public class CalculateShooterCurvature extends OpMode {
    private Limelight3A limelight;
    private IMU imu;
    Servo coverL, coverR, block;
    double distance = 0;
    double F = 1;
    double steps = 0.1;
    int stepsVel = 50;
    DcMotorEx shooter1, shooter2;
    DcMotorEx  intake, transfer;
    double highVel = 1600;
    double currVel = highVel;
    ColorSensor colorSensor;
    ColorSensor.DetectedColors detectedColor;
    DriveTrain driveTrain;
    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients coeficients = new PIDFCoefficients(1,0,0,15.3);
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
        block = hardwareMap.get(Servo.class,"block");
        coverL.setDirection(Servo.Direction.REVERSE);
        colorSensor = new ColorSensor();
        colorSensor.init(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);
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
        driveTrain.TeleOp(gamepad1,telemetry, orientation.getYaw());
        if (llResult != null && llResult.isValid()){
            distance = getDistanceFromTargeta(llResult.getTa());
        }

        if(gamepad1.dpadLeftWasPressed())
            F += steps;
        if(gamepad1.dpadRightWasPressed())
            F -= steps;

        if(gamepad1.dpadUpWasPressed())
            currVel += stepsVel;
        if(gamepad1.dpadDownWasPressed())
            currVel -= stepsVel;

        if(gamepad1.bWasPressed())
            block.setPosition(1);

        if(gamepad1.xWasPressed())
            block.setPosition(0);

        detectedColor = colorSensor.getDetectedColor(telemetry);

        if(detectedColor != ColorSensor.DetectedColors.UNKNOWN)
            transfer.setPower(0);
        //F = getPosCover(distance);
        coverL.setPosition(F);
        coverR.setPosition(F);

        if(gamepad1.leftBumperWasPressed()){
            intake.setPower(1);
            transfer.setPower(1);
        }
        if(gamepad1.leftBumperWasPressed() && detectedColor != ColorSensor.DetectedColors.UNKNOWN)
            transfer.setPower(1);

        if (gamepad1.rightBumperWasPressed()){
            intake.setPower(0);
            transfer.setPower(0);
        }
        double vel = getVelocity(distance);
        if(gamepad1.left_trigger_pressed){
            if(gamepad1.right_trigger_pressed) {
                if(block.getPosition()!=0.5)
                    block.setPosition(0.5);
                else {
                    intake.setPower(1);
                    transfer.setPower(1);
                }
            }else{
                block.setPosition(1);
                intake.setPower(0);
                transfer.setPower(0);
            }
            if(vel - shooter1.getVelocity() > 100){
                shooter1.setPower(1);
                shooter2.setPower(1);
            }else {
                shooter1.setVelocity(vel);
                shooter2.setVelocity(vel);
            }
        }else{
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
        telemetry.addData("Position",coverR.getPosition());
        telemetry.addData("Distance",distance);
        telemetry.addData("Velocity", vel);
        telemetry.addData("Pos", F);
        telemetry.addData("RealVelocity",shooter1.getVelocity());
        telemetry.addData("RealVelocity",shooter2.getVelocity());
        telemetry.update();
    }
    private double getPosCover(double distance){
        return Math.min(0,Math.max(-0.0027027 * distance + 1.082164,1));
    }
    private double getVelocity(double x){
        return (int)(157.4115 +
                16.39278 * x -
                0.08673005 * Math.pow(x,2) +
                0.0001515039 * Math.pow(x,3));
    }
    private double getDistanceFromTargeta(double ta){
        return 180.5062* Math.pow(ta,-0.5018798);
    }
}
