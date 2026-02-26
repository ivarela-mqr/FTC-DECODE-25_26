package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.mechanisms.ColorSensor;
import org.firstinspires.ftc.teamcode.mechanisms.DriveTrain;

import java.security.Provider;
@TeleOp
@Config
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
    public static double highVel = 1200;
    public static double currVel = 1200;
    ColorSensor colorSensor;
    ColorSensor.DetectedColors detectedColor;
    DriveTrain driveTrain;
    public static double p,i,d,f;
    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter0");

        p = 0;
        i = 0;
        d = 0;
        f = 0;
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

    }

    @Override
    public void start(){
        limelight.start();
    }
    @Override
    public void loop() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("CurrVel", Math.max(shooter1.getVelocity(),shooter2.getVelocity()));
        dashboardTelemetry.addData("Target Vel", currVel);
        dashboardTelemetry.addData("Limit",0);
        dashboardTelemetry.update();
        PIDFCoefficients coeficients = new PIDFCoefficients(p,i,d,f);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);

        if(gamepad1.right_trigger > 0.1){
            if(Math.abs(currVel - Math.max(shooter2.getVelocity(),shooter1.getVelocity())) > 100){
                shooter1.setPower(1);
                shooter2.setPower(1);
            }else{
                shooter1.setVelocity(currVel);
                shooter2.setVelocity(currVel);
            }
        }else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

        telemetry.addData("Velocity", currVel);
        telemetry.addData("RealVelocity",shooter1.getVelocity());
        telemetry.addData("RealVelocity",shooter2.getVelocity());
        telemetry.addData("P",p);
        telemetry.addData("I",i);
        telemetry.addData("D",d);
        telemetry.addData("F",f);
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
