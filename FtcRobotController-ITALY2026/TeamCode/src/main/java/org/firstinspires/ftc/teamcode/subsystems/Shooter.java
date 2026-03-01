package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debouncer;

public class Shooter {
    public  DcMotorEx shooter0, shooter1, encoder;
    public CRServo rotorL, rotorR;
    public Servo coverL, coverR, block;
    public LimeLight limeLight;
    int LEFT_LIMIT = -10000;
    int RIGHT_LIMIT = 10000;
    final double VELOCITY_FACTOR = 0.1;
    public double offset = 0, correctOffset = 0;
    double curTargetVelocity;
    public Timer init = new Timer();
    public boolean autoAim = true, teleOp = false;
    public Debouncer debouncer = new Debouncer(200);
    public Debouncer velDebouncer = new Debouncer(200);
    public Debouncer blockDebouncer = new Debouncer(200);
    PIDFCoefficients coefficients = new PIDFCoefficients(22, 0, 1.7, 15);
    private double lastValidOffset = 0;
    public boolean stop = false;
    private final ElapsedTime timer = new ElapsedTime();
    public double kP = 0.35;
    public double kD = 0.05;
    double lastError = 0;
    double lastTime = 0;
    boolean hold = true;
    public Shooter (HardwareMap hardwareMap, Constants.Alliance alliance, double targetVel){
        shooter0 = hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        rotorL = hardwareMap.get(CRServo.class,"rotorL");
        rotorR = hardwareMap.get(CRServo.class,"rotorR");
        coverL = hardwareMap.get(Servo.class,"coverL");
        coverR = hardwareMap.get(Servo.class,"coverR");
        block = hardwareMap.get(Servo.class, "block");
        shooter0.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coverL.setDirection(Servo.Direction.REVERSE);
        limeLight = new LimeLight(hardwareMap, alliance);
        curTargetVelocity = targetVel;
        timer.reset();
    }
    public void resetTimer(){
        init.resetTimer();
    }
    public void adjustVelAndCover(double distance){
        if(distance > 0 && teleOp && autoAim){
            curTargetVelocity = 744.8561 + 4.052032*distance - 0.005816 * Math.pow(distance,2);
            double pos = 1.212238 - 0.01040305*distance + 0.00002366859 * Math.pow(distance,2);
            adjustCover(pos);
        }
    }
    public void aimWithLimelight(double yaw){
        double[] var = limeLight.getGoalAprilTagData(yaw);
        offset = var[0];
        adjustVelAndCover(var[1]);
        moveServos(offset, offset != 0);
    }
    public double pid(double offset){
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        offset += correctOffset;
        double derivative = 0;
        if (dt > 0) {
            derivative = (offset - lastError) / dt;
        }
        double output = (kP * offset)
                + (kD * derivative);
        lastError = offset;
        if (!hold && Math.abs(offset) < 3){
            output = 0;
            hold = true;
        }else if(hold && Math.abs(offset) > 5){
            hold = false;
        }
        output = Math.max(-1, Math.min(1, output * VELOCITY_FACTOR));
        return -output;
    }
    public void moveServos(double offsetX, boolean objectDetected) {
        if (objectDetected)
            lastValidOffset = offsetX;
        else
            offsetX = lastValidOffset;
        double power = pid(offsetX);
        int posR = encoder.getCurrentPosition();
        if (autoAim) {
            setPowerRotor(power);
        }
        if ((posR <= LEFT_LIMIT && power > 0) ||
                (posR >= RIGHT_LIMIT && power < 0)) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }
    }
    public void setPowerRotor(double power){
        int posR = encoder.getCurrentPosition();
        if ((posR <= LEFT_LIMIT && power > 0) ||
                (posR >= RIGHT_LIMIT && power < 0)) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }else{
            rotorL.setPower(power);
            rotorR.setPower(power);
        }
    }
    public void resetRotorPosition(){
        if(encoder.getCurrentPosition()>25)
            setPowerRotor(5 * VELOCITY_FACTOR);
        else if(encoder.getCurrentPosition()<-25)
            setPowerRotor(-5 * VELOCITY_FACTOR);
        else
            setPowerRotor(0);
    }
    public boolean isReady(){
        return velocityOffset() < 50
                && offset < 5;
    }
    public boolean canShoot(Gamepad gamepad){
        return (velocityOffset() < 50
                && block.getPosition() < 0.1) || gamepad.left_trigger > 0.1;
    }
    public void adjustCover(double dist){
        coverL.setPosition(dist);
        coverR.setPosition(dist);
    }
    public void correctCover(double direction){
        coverL.setPosition(coverL.getPosition()+(direction*0.05));
        coverR.setPosition(coverR.getPosition()+(direction*0.05));
    }
    public void preload() {
        if (velocityOffset() > 100) {
            shooter0.setPower(1);
            shooter1.setPower(1);
        } else {
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
        if(velocityOffset() > 100){
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
    public void startTeleop(){
        teleOp = true;
        RIGHT_LIMIT = Constants.RIGHT_TELEOP_LIMIT;
        LEFT_LIMIT = Constants.LEFT_TELEOP_LIMIT;
    }
    public double velocityOffset(){
        return curTargetVelocity - Math.max(shooter1.getVelocity(), shooter0.getVelocity());
    }
    public void TeleOp(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry,
                       double yawAngle, boolean isFull){
        aimWithLimelight(yawAngle);
        if(gamepad1.dpadUpWasPressed())
            stop = true;
        if(!stop)
            preload();
        else
            stop();
        if(isFull && (isReady() || gamepad1.left_trigger > 0.1) && gamepad1.right_trigger > 0.1)
            openBlock();
        else if(gamepad1.right_trigger < 0.9 || gamepad1.left_trigger < 0.9)
            closeBlock();

        if (gamepad2.share)
            autoAim = true;
        if (gamepad2.dpad_left)
            limeLight.switchAlliance(Constants.Alliance.BLUE);
        else if(gamepad2.dpad_right)
            limeLight.switchAlliance(Constants.Alliance.RED);

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

        if(gamepad2.dpad_up && velDebouncer.isReady())
            curTargetVelocity += 50;
        if(gamepad2.dpad_down && velDebouncer.isReady())
            curTargetVelocity -= 50;
        if(gamepad2.left_bumper && debouncer.isReady())
            correctCover(-1);
        if(gamepad2.right_bumper && debouncer.isReady())
            correctCover(1);
        if (gamepad2.circle && blockDebouncer.isReady())
            switchBlock();

        telemetry.addData("velocity shooter",shooter0.getVelocity());
        telemetry.addData("curTargetVelocity",curTargetVelocity);
        telemetry.addData("cover pos", coverR.getPosition());
    }
}
