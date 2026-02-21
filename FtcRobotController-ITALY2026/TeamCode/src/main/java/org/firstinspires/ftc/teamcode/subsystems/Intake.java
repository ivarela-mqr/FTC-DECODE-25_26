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
    DcMotorEx transfer;
    DcMotor intake;
    DistanceSensor distanceSensor1, distanceSensor2;
    ColorSensor colorSensor = new ColorSensor();
    boolean isShooting = false, isIntaking = false;

    Debouncer crossDebouncer = new Debouncer(200);

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distanceSensor2");
        colorSensor.init(hardwareMap);

    }

    public boolean getIsShooting() {
        return isShooting;
    }

    public void intakeFirstArtifact() {
        transfer.setPower(1);
        intake.setPower(1);
    }

    public void intakeNextArtifacts() {
        transfer.setPower(0);
        intake.setPower(1);
    }

    public void unload() {
        intake.setPower(-0.2);
        transfer.setPower(-0.5);
    }

    public void shoot1Artifact() {

    }

    public void shootArtifacts() {
        intake.setPower(1);
        transfer.setVelocity(6000);
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

    public boolean firstArtifactIn() {
        return colorSensor.getDetectedColor() != ColorSensor.DetectedColors.UNKNOWN;
    }

    public boolean secondArtifactIn() {
        return distanceSensor2.getDistance(DistanceUnit.CM) < 8;
    }

    public boolean thirdArtifactIn() {
        return distanceSensor1.getDistance(DistanceUnit.CM) < 12.5;
    }


    public void intake() {
       transfer.setPower(0.75);
       intake.setPower(1);
    }



    public void TeleOp(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry, double yawAngle) {

        if (isIntaking) {
            intake();
        } else {
            stopArtifacts();
        }

        if (gamepad1.cross && crossDebouncer.isReady()) {
            isIntaking = !isIntaking;
        }

        if (gamepad1.triangle){
            unload();
        }

        //shoot
        if (gamepad1.right_trigger > 0.1) {
            isShooting = true;
        } else {
            isShooting = false;
        }


        //stoppers
        if (gamepad2.square) {
            isShooting = false;
        }
        if (gamepad2.triangle) {
            isShooting = false;
            isIntaking = false;
        }
    }
}