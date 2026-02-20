package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
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
import org.opencv.core.Mat;


public class Shooter {
   public  DcMotorEx shooter0, shooter1;
   public CRServo rotorL, rotorR;
   Servo coverL, coverR, block;
   LimeLight limeLight;
   DcMotorEx encoder;
   final int LIMITE_IZQUIERDA = -10000;
   final int LIMITE_DERECHA = 15000;
    final double VELOCIDAD_FACTOR = - 0.05;
    double offset = 0;
    double highVelocityShooter = 1500;
    double lowVelocityShooter = 500;
    double curTargetVelocity = 1200;
    double kP_shooter = 1.25;
    double kI_shooter = 0;
    double kD_shooter = 0;
    double kF_shooter = 17.5;
    Timer init;
    public boolean autoAim = true;

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
        coverL.setPosition(targetAngle);
        coverR.setPosition(targetAngle);
        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        init = new Timer();
    }
    public void initTimer(){
        init.resetTimer();
    }
    public void aimWithLimelight(double yaw){
        offset = limeLight.getGoalAprilTagData(yaw)[0];
        moverServos(offset);
    }
    public void moverServos(double offsetX) {
        int posR = encoder.getCurrentPosition();

        // Calculamos potencia a partir del offset
        double potencia = offsetX * VELOCIDAD_FACTOR;
        if(autoAim) {
            // Lectura de encoders

            // Verificamos límites: si cualquiera está en el límite, paramos ambos
            if ((posR <= LIMITE_IZQUIERDA && potencia > 0) ||
                    (posR >= LIMITE_DERECHA && potencia < 0)) {
                potencia = 0;
            }

            // Aplicamos misma potencia a ambos servos
            rotorL.setPower(potencia);
            rotorR.setPower(potencia);
        }
        // Verificamos límites: si cualquiera está en el límite, paramos ambos
        if ((posR <= LIMITE_IZQUIERDA && potencia > 0) ||
                (posR >= LIMITE_DERECHA && potencia < 0)) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }
    }

    public void setPowerRotor(double power){
        rotorL.setPower(power * VELOCIDAD_FACTOR);
        rotorR.setPower(power * VELOCIDAD_FACTOR);
    }

    public boolean isReady(){
        return Math.abs(curTargetVelocity - Math.max(shooter1.getVelocity(),shooter0.getVelocity())) < 50;
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
        //double distance = limeLight.getGoalAprilTagData(telemetry, yawAngle)[1];
        /*if(distance != 0)
            curTargetVelocity = getVelocity(distance);
        else
            curTargetVelocity = 1000;*/
        curTargetVelocity = 1200;

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
            shooter0.setPower(0);
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
    private double getVelocity(double x){
        return (int)(157.4115 +
                16.39278 * x -
                0.08673005 * Math.pow(x,2) +
                0.0001515039 * Math.pow(x,3));
    }
    public void TeleOp(Gamepad gamepad, Telemetry telemetry, double yawAngle, boolean isShooting){
        //adjust shooter position
        double[] tx = limeLight.getGoalAprilTagData(yawAngle);
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
