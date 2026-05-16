package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Intake {
    public DcMotorEx transfer;
    public DcMotor intake;
    public LightSensor lightSensor1, lightSensor2, lightSensor3;
    public double transferPosition2ArtifactIn;
    public Servo led;


    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        lightSensor1 = new LightSensor(hardwareMap, "lSensor11", "lSensor12");
        lightSensor2 = new LightSensor(hardwareMap, "lSensor21", "lSensor22");
        lightSensor3 = new LightSensor(hardwareMap, "lSensor31", "lSensor32");
        led = hardwareMap.get(Servo.class, "led");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void intakeFirstArtifact() {
        transfer.setPower(1);
        intake.setPower(1);
    }
    public void intakeNextArtifacts() {
        transfer.setPower(0);
        intake.setPower(1);
    }
    public void demo(){
        transfer.setPower(0.5);
        intake.setPower(0.5);
    }
    public void unload() {
        intake.setPower(-0.2);
        transfer.setPower(-0.5);
    }
    public void shootArtifacts() {
        intake.setPower(1);
        transfer.setPower(1);
    }
    public void stopArtifacts() {
        intake.setPower(0);
        transfer.setPower(0);
    }
    public int numArtifactsIn() {
        int num = 0;
        if (firstArtifactIn())
            num += 1;
        if (secondArtifactIn())
            num += 1;
        if (thirdArtifactIn())
            num += 1;
        return num;
    }
    public void setTransferPosition(double targetRelativePosition) {
        if (Math.abs(transfer.getCurrentPosition() - transferPosition2ArtifactIn) < targetRelativePosition) {
            transfer.setPower(0.1);
        }
    }
    public boolean firstArtifactIn() {
        return lightSensor1.isDetecting();
    }
    public boolean secondArtifactIn() { return lightSensor2.isDetecting();}
    public boolean thirdArtifactIn() {
        return lightSensor3.isDetecting();
    }

}