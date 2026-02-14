package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Shooter {
    DcMotorEx shooter0, shooter1;
    Servo rotorL, rotorR, coverL, coverR, block;

    LimeLight limeLight;


    // PID constants rotors
    double kP_rotor = 0.02;
    double kI_rotor = 0.0;
    double kD_rotor = 0.002;

    // PID state rotors
    double integralSum_rotor = 0;
    double lastError_rotor = 0;
    long lastTime_rotor = 0;

    // Servo state rotors
    double rotorPos = 0.5;


    //PIDF shooter
    double highVelocityShooter = 1500;
    double lowVelocityShooter = 500;

    double curTargetVelocity = 1200;

    double kP_shooter = 1;
    double kI_shooter = 0;
    double kD_shooter = 0;
    double kF_shooter = 15.4;



    public Shooter (HardwareMap hardwareMap, Constants.Alliance alliance){
        shooter0 = hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        rotorL = hardwareMap.get(Servo.class,"rotorL");
        rotorR = hardwareMap.get(Servo.class,"rotorR");
        coverL = hardwareMap.get(Servo.class,"coverL");
        coverR = hardwareMap.get(Servo.class,"coverR");
        block = hardwareMap.get(Servo.class, "block");

        shooter0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients_shooter = new PIDFCoefficients(kP_shooter, kI_shooter, kD_shooter, kF_shooter);

        shooter0.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients_shooter);
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients_shooter);

        coverL.setDirection(Servo.Direction.REVERSE);

        limeLight = new LimeLight(hardwareMap, alliance);

    }

    private double pidCalculate(double error) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime_rotor) / 1e9;

        if (lastTime_rotor == 0) {
            lastTime_rotor = currentTime;
            return 0;
        }

        integralSum_rotor += error * dt;
        double derivative = (error - lastError_rotor) / dt;

        lastError_rotor = error;
        lastTime_rotor = currentTime;

        return (kP_rotor * error) + (kI_rotor * integralSum_rotor) + (kD_rotor * derivative);
    }
    public void resetPID() {
        integralSum_rotor = 0;
        lastError_rotor = 0;
        lastTime_rotor = 0;
    }

    public void aimShooterWithLimeLight(Telemetry telemetry,double angle){


        // If AprilTag not detected don't move
        if (Math.abs(angle) < 0.05) return;

        double correction = pidCalculate(angle);

        rotorPos -= correction;

        // Limit servo position
        rotorPos = Math.max(0.0, Math.min(1.0, rotorPos));

        rotorL.setPosition(rotorPos);
        rotorR.setPosition(rotorPos);

        telemetry.addLine("AimShooterWithLimeLight:");
        telemetry.addData("Tx", angle);
        telemetry.addData("PID Correction", correction);
        telemetry.addData("Rotor Pos", rotorPos);
    }

    public void aimShooterWithOdometry(){

    }

    public boolean isReady(){
        return (curTargetVelocity - shooter0.getVelocity() < 150) && block.getPosition() == 0.5;
    }
    public void adjustCover(double dist){
        coverL.setPosition(dist);
        coverR.setPosition(dist);
    }

    public void correctCover(Telemetry telemetry, double direction){
        coverL.setPosition(coverL.getPosition()+(direction*0.05));
        coverR.setPosition(coverR.getPosition()+(direction*0.05));
    }

    public void switchCurTargetVelocity(){
        if (curTargetVelocity == highVelocityShooter) {
            curTargetVelocity = lowVelocityShooter;
        }else{curTargetVelocity = highVelocityShooter;}
    }

    public void setShootingPower(Telemetry telemetry) {
        if (Math.abs(shooter0.getVelocity() - curTargetVelocity) > 100) {

        }else{
            shooter0.setVelocity(curTargetVelocity);
            shooter1.setVelocity(curTargetVelocity);
        }
        telemetry.addData("Shooter velocity",shooter0.getVelocity());
    }
    public void preload(Telemetry telemetry, double yawAngle) {
        double distance = limeLight.getGoalAprilTagData(telemetry, yawAngle)[1];
        if(distance != 0)
            curTargetVelocity = getVelocity(distance);
        else
            curTargetVelocity = 1000;
        if(curTargetVelocity - shooter1.getVelocity() > 100){
            shooter0.setPower(1);
            shooter1.setPower(1);
        }else {
            shooter0.setVelocity(curTargetVelocity);
            shooter1.setVelocity(curTargetVelocity);
        }
    }
    public void closeBlock() {
        block.setPosition(1);
    }
    public double getBlockPos() {
        return block.getPosition();
    }
    public void calm(){
        shooter0.setVelocity(900);
        shooter1.setVelocity(900);
    }
    public void stop() {
        shooter0.setPower(0);
        shooter1.setPower(0);
    }
    public void openBlock(){
        block.setPosition(0.5);
    }
    private double getVelocity(double x){
        return (int)(157.4115 +
                16.39278 * x -
                0.08673005 * Math.pow(x,2) +
                0.0001515039 * Math.pow(x,3));
    }
    public void TeleOp(Gamepad gamepad, Telemetry telemetry, double yawAngle, boolean isShooting){
        //adjust shooter position
        double[] tx = limeLight.getGoalAprilTagData(telemetry, yawAngle);
        //aimShooterWithLimeLight(telemetry, tx[0]);
        //adjustCover(telemetry, tx[1]);

        //correct shooter cover
        if(gamepad.left_bumper){
            correctCover(telemetry, -1);
        }else if(gamepad.right_bumper){
            correctCover(telemetry, 1);
        }
        telemetry.addData("Shooting shooter?",isShooting);
        //shoot motor power
        if(isShooting){
            curTargetVelocity = highVelocityShooter;
        }else {curTargetVelocity = lowVelocityShooter;}

        setShootingPower(telemetry);

        //telemetry.addData("cover position: ", coverL.getPosition());
    }




}
