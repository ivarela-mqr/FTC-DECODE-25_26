package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Debouncer;

public class Intake {
    DcMotorEx  intake, transfer;
    //Servo block;
    //DistanceSensor distanceSensor1, distanceSensor2;
    ColorSensor colorSensor;
    boolean isShooting = false, isIntaking = false;

    Debouncer crossDebouncer = new Debouncer(200);


    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //block = hardwareMap.get(Servo.class, "block");

        /*distanceSensor1 = hardwareMap.get(DistanceSensor.class,"distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class,"distanceSensor2");*/
        colorSensor.init(hardwareMap);

    }

    public boolean getIsShooting(){
        return isShooting;
    }

    public void intakeArtifact(Telemetry telemetry){
        //isShooting = false;
        if(!firstArtifactIn(telemetry)){
            transfer.setPower(0.75);
        }else{
            transfer.setPower(0);
        }
        if(!secondArtifactIn() || !thirdArtifactIn()){
            intake.setPower(0.75);
        }
    }
    public void shoot1Artifact(){

    }
    public void shootArtifacts(){
        intake.setPower(0.75);
        transfer.setPower(0.75);
    }
    public void stopArtifacts(){
        intake.setPower(0);
        transfer.setPower(0);
    }

    int numArtifactsIn(Telemetry telemetry){
        int num = 0;
        if(firstArtifactIn(telemetry))
            num+=1;
        if(secondArtifactIn())
            num+=1;
        if(thirdArtifactIn())
            num+=1;
        return num;
    }
    boolean firstArtifactIn(Telemetry telemetry){
        return colorSensor.getDetectedColor(telemetry)!= ColorSensor.DetectedColors.UNKNOWN;
    }
    boolean secondArtifactIn(){
        return false;//distanceSensor2.getDistance(DistanceUnit.CM)<15;
    }
    boolean thirdArtifactIn(){
        return false; //distanceSensor1.getDistance(DistanceUnit.CM)<15;
    }

    public void switchBlock(){
        /*if (block.getPosition()>0.8){
            block.setPosition(0);
        }else{block.setPosition(1);}*/
    }


    public void TeleOp(Gamepad gamepad, Telemetry telemetry, double yawAngle){
        if (isShooting){
            //block.setPosition(1);
            shootArtifacts();
        }else{
            //block.setPosition(0);
            stopArtifacts();
        }

        if(isIntaking){
            intakeArtifact(telemetry);
        }else{
            stopArtifacts();
        }

        if(gamepad.cross&&crossDebouncer.isReady()){
            isIntaking = !isIntaking;
        }
//        if(numArtifactsIn(telemetry) == 0){
//            //isShooting = false;
//        }else
        if(gamepad.right_trigger>0.1){
            isShooting = true;
        }else{
            isShooting = false;
        }
//        if(numArtifactsIn(telemetry) == 3) {
//            isIntaking = false;
//        }
    }
}