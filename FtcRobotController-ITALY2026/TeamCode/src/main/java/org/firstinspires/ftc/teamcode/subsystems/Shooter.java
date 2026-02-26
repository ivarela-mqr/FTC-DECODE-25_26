package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debouncer;

public class Shooter {
    public  DcMotorEx shooter0, shooter1;
    public CRServo rotorL, rotorR;
    public Servo coverL, coverR, block;
    public LimeLight limeLight;
    public DcMotorEx encoder;
    int LEFT_LIMIT = -10000;
    final int LEFT_TELEOP_LIMIT = -2500;
    int RIGHT_LIMIT = 10000;
    final int RIGHT_TELEOP_LIMIT = 2500;
    final double VELOCITY_FACTOR = - 0.05;
    double offset = 0;
    double curTargetVelocity = 1200;
    double kP_shooter = 1.15;
    double kI_shooter = 0.0005;
    double kD_shooter = 1.35;
    double kF_shooter = 16;
    public Timer init;
    public boolean autoAim = true;
    public Debouncer debouncer = new Debouncer(200);
    public Debouncer velDebouncer = new Debouncer(200);
    public Debouncer blockDebouncer = new Debouncer(200);
    private double lastValidOffset = 0;
    private boolean teleOp = false;
    public Shooter (HardwareMap hardwareMap, Constants.Alliance alliance, double targetVel, double targetAngle){
        shooter0 = hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        rotorL = hardwareMap.get(CRServo.class,"rotorL");
        rotorR = hardwareMap.get(CRServo.class,"rotorR");
        coverL = hardwareMap.get(Servo.class,"coverL");
        coverR = hardwareMap.get(Servo.class,"coverR");
        block = hardwareMap.get(Servo.class, "block");


        PIDFCoefficients pidfCoefficients_shooter = new PIDFCoefficients(kP_shooter, kI_shooter, kD_shooter, kF_shooter);

        shooter0.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients_shooter);
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients_shooter);

        coverL.setDirection(Servo.Direction.REVERSE);

        limeLight = new LimeLight(hardwareMap, alliance);
        curTargetVelocity = targetVel;
        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        init = new Timer();
    }
    public void initTimer(){
        init.resetTimer();
    }
    public void adjustVelAndCover(double distance){
        if(distance > 0 && teleOp){
            curTargetVelocity = 1.685393*distance + 930.3371;
            double pos =  -0.000921659*distance + 0.4115207;
            adjustCover(pos);
        }
    }
    public void aimWithLimelight(double yaw, Telemetry telemetry){
        double[] var = limeLight.getGoalAprilTagData(yaw);
        offset = var[0];
        adjustVelAndCover(var[1]);
        moveServos(offset, offset != 0);
    }
    public void moveServos(Double offsetX, boolean objectDetected) {
        int posR = encoder.getCurrentPosition();

        if (objectDetected) {
            lastValidOffset = offsetX;
        } else {
            offsetX = lastValidOffset;
        }

        double power = offsetX * VELOCITY_FACTOR;

        if (autoAim) {
            if ((posR <= LEFT_LIMIT && power > 0) ||
                    (posR >= RIGHT_LIMIT && power < 0)) {
                power = 0;
            }

            rotorL.setPower(power);
            rotorR.setPower(power);
        }

        if ((posR <= LEFT_LIMIT && power > 0) ||
                (posR >= RIGHT_LIMIT && power < 0)) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }
    }

    public void setPowerRotor(double power){
        int posR = encoder.getCurrentPosition();
        if ((posR <= LEFT_LIMIT && rotorR.getPower() > 0) ||
                (posR >= RIGHT_LIMIT && rotorL.getPower() < 0)) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }else{
            rotorL.setPower(power * VELOCITY_FACTOR);
            rotorR.setPower(power * VELOCITY_FACTOR);
        }
    }
    public void resetRotorPosition(){
        if(encoder.getCurrentPosition()>50){
            setPowerRotor(-20);
        }else if(encoder.getCurrentPosition()<-50){
            setPowerRotor(20);
        }else{
            setPowerRotor(0);
        }
    }

    public boolean isReady(){
        return curTargetVelocity - Math.max(shooter1.getVelocity(),shooter0.getVelocity()) < 50;
    }
    public boolean canShoot(){
        return curTargetVelocity - Math.max(shooter1.getVelocity(),shooter0.getVelocity()) < 50 && block.getPosition() <= 0.2;
    }
    public void adjustCover(double dist){
        coverL.setPosition(dist);
        coverR.setPosition(dist);
    }

    public void correctCover(Telemetry telemetry, double direction){
        coverL.setPosition(coverL.getPosition()+(direction*0.05));
        coverR.setPosition(coverR.getPosition()+(direction*0.05));
    }
    public void preload(Telemetry telemetry, double yawAngle) {
        if(curTargetVelocity - Math.max(shooter1.getVelocity(),shooter0.getVelocity()) > 150){
            shooter0.setPower(1);
            shooter1.setPower(1);
        }else {
            shooter0.setVelocity(curTargetVelocity);
            shooter1.setVelocity(curTargetVelocity);
        }
    }
    public void closeBlock() {
        block.setPosition(0.7);
    }
    public double getBlockPos() {
        return block.getPosition();
    }
    public void calm(){
        if(Math.abs(curTargetVelocity - Math.max(shooter1.getVelocity(), shooter0.getVelocity())) > 100){
            shooter0.setPower(1);
            shooter1.setPower(1);
        }else {
            shooter0.setVelocity(curTargetVelocity);
            shooter1.setVelocity(curTargetVelocity);
        }
    }
    public void stop() {
        shooter0.setPower(0);
        shooter1.setPower(0);
    }
    public void openBlock(){
        block.setPosition(0);
    }
    public void switchBlock(){
        if (block.getPosition()>0.5){
            block.setPosition(0);
        }else{block.setPosition(1);}
    }
    public void TeleOp(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, double yawAngle, boolean isFull){
        //aim rotor
        if (gamepad2.share){
            autoAim = true;
        }
        teleOp = true;
        RIGHT_LIMIT = RIGHT_TELEOP_LIMIT;
        LEFT_LIMIT = LEFT_TELEOP_LIMIT;
        aimWithLimelight(yawAngle,telemetry);

        //correct shooter rotor
        if (gamepad2.left_trigger > 0.1){
            autoAim = false;
            setPowerRotor(-10);
        }else if (gamepad2.right_trigger > 0.1) {
            autoAim = false;
            setPowerRotor(10);
        } else if (!autoAim){
            setPowerRotor(0);
            autoAim = false;
        }

        if(isFull){
            preload(telemetry,yawAngle);
            if(isReady() && gamepad1.right_trigger > 0.1){
                openBlock();
            }
        }else {
            calm();
            closeBlock();
        }

        if(gamepad2.dpad_up && velDebouncer.isReady())
            curTargetVelocity += 50;

        if(gamepad2.dpad_down && velDebouncer.isReady())
            curTargetVelocity -= 50;


        //correct shooter cover
        if(gamepad2.left_bumper && debouncer.isReady()){
            correctCover(telemetry, -1);
        }
        if(gamepad2.right_bumper && debouncer.isReady()){
            correctCover(telemetry, 1);
        }

        if (gamepad2.circle && blockDebouncer.isReady()){
            switchBlock();
        }


        telemetry.addData("velocity shooter",shooter0.getVelocity());
        telemetry.addData("curTargetVelocity",curTargetVelocity);
        telemetry.addData("cover pos", coverR.getPosition());
    }
}
