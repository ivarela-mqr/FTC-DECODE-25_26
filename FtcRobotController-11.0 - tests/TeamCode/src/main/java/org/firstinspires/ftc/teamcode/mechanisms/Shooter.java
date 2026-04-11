package org.firstinspires.ftc.teamcode.mechanisms;

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

public class Shooter {
    public  DcMotorEx shooter0, shooter1, encoder;
    public CRServo rotorL, rotorR;
    public Servo coverL, coverR, block;
    int LEFT_LIMIT = -10000;
    int RIGHT_LIMIT = 10000;
    final double VELOCITY_FACTOR = 0.1;
    public double offset = 0, correctOffset = 0;
    double curTargetVelocity;
    public boolean autoAim = true, teleOp = false;
    PIDFCoefficients coefficients = new PIDFCoefficients(22, 0, 1.7, 15);
    private double lastValidOffset = 0;
    public boolean stop = false;
    private final ElapsedTime timer = new ElapsedTime();
    public double kP = 0.35;
    public double kD = 0.05;
    double lastError = 0;
    double lastTime = 0;
    public boolean hold = true, reset = false;
    double pos;
    public Shooter (HardwareMap hardwareMap, double targetVel){
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
        curTargetVelocity = targetVel;
        timer.reset();

        openBlock();
    }
    public void adjustVelAndCover(double distance){
        if(distance > 0 && teleOp && autoAim){
            double pos = 0.15;
            curTargetVelocity = 1550;
            if (distance < 300){
                pos = 1.31952 - 0.01056727*distance + 0.00002452835 * Math.pow(distance,2);
                curTargetVelocity = 752.1383 + 3.887819*distance - 0.004956243 * Math.pow(distance,2);
            }
            adjustCover(pos);
        }
    }

    /*public void aimWithOdometry(double yaw) {
        // Posición actual del robot desde Pedro Pathing
        Pose robotPose = follower.getPose();

        // Coordenadas de la canasta según alianza
        double basketX, basketY;
        if (Constants.Alliance.BLUE == alliance) {
            basketX = BLUE_BASKET_X;
            basketY = BLUE_BASKET_Y;
        } else {
            basketX = RED_BASKET_X;
            basketY = RED_BASKET_Y;
        }

        // Vector robot → canasta
        double dx = basketX - robotPose.getX();
        double dy = basketY - robotPose.getY();

        // Distancia (equivalente a var[1] de limelight)
        double distance = Math.hypot(dx, dy);

        // Ángulo absoluto al campo hacia la canasta
        double angleToBasket = Math.atan2(dy, dx);

        // Ángulo relativo a la orientación del robot (usando yaw igual que limelight)
        double turretTarget = Math.toDegrees(angleToBasket) - yaw;

        // Normalizar a [-180, 180]
        while (turretTarget >  180) turretTarget -= 360;
        while (turretTarget < -180) turretTarget += 360;

        // Offset: diferencia entre donde está la torreta y donde debe estar
        // (igual que limelight devuelve tx como desviación)
        offset = turretTarget - currentTurretAngle;

        // Mismo correctOffset que en limelight, basado en distancia y alianza
        if (distance > 300 && Constants.Alliance.BLUE == alliance)
            correctOffset = -5;
        else if (distance > 300 && Constants.Alliance.RED == alliance)
            correctOffset = 5;
        else
            correctOffset = 0;

        // Reutiliza exactamente los mismos métodos que limelight
        adjustVelAndCover(distance);

        boolean offsetCentered = (offset == 0 && Math.abs(lastValidOffset) < 5);
        moveServos(offset, !offsetCentered);
    } */
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
        output = Math.max(-0.3, Math.min(0.3, output * VELOCITY_FACTOR));
        return -output;
    }
    public void moveServos(double offsetX, boolean objectDetected) {
        if (objectDetected)
            lastValidOffset = offsetX;
        else
            offsetX = lastValidOffset;
        double power = pid(offsetX);
        int posR = encoder.getCurrentPosition();
        if (!objectDetected) {
            if (posR > 500) {
                setPowerRotor(0.1);
                return;
            } else if (posR < -500) {
                setPowerRotor(-0.1);
                return;
            }
        }
        if (autoAim) {
            setPowerRotor(power);
        }
        if (!reset && (posR <= LEFT_LIMIT && power > 0) ||
                (posR >= RIGHT_LIMIT && power < 0)) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }
    }
    public void setPowerRotor(double power){
        int posR = encoder.getCurrentPosition();
        if (!reset && (posR <= LEFT_LIMIT && power > 0) ||
                (posR >= RIGHT_LIMIT && power < 0)) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }else{
            rotorL.setPower(power);
            rotorR.setPower(power);
        }
    }
    public boolean isReady(){
        return velocityOffset() < 50;
    }
    public boolean isReady2(){
        return offset < 7.5 && offset > - 7.5;
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
        block.setPosition(1);
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
        if (block.getPosition()>0.8){
            block.setPosition(0);
        }else{block.setPosition(1);}
    }
    public double velocityOffset(){
        return curTargetVelocity - Math.max(shooter1.getVelocity(), shooter0.getVelocity());
    }
    public void stopTurret(){
        rotorL.setPower(0);
        rotorR.setPower(0);
    }
    public void TeleOp(double vel, double pos){
        curTargetVelocity = vel;
        shooter0.setVelocity(curTargetVelocity);
        shooter1.setVelocity(curTargetVelocity);

        adjustCover(pos);
    }
}
