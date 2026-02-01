package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DriveTrain {
    //variables
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private double drive = 0;
    private double strafe = 0;
    private double rotate = 0;

    DriveTrain(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void forwardMovement() {}

    public void strafingMovement(){}

    public void rotatingMovement(){}

    public void TeleOp(Gamepad gamepad, Telemetry telemetry, double yawAngle){
        drive = -gamepad.left_stick_y;
        strafe = gamepad.left_stick_x;
        rotate = -gamepad.right_stick_x;


        double adjustedDrive = drive * Math.cos(Math.toRadians(yawAngle)) - strafe * Math.sin(Math.toRadians(yawAngle));
        double adjustedStrafe = drive * Math.sin(Math.toRadians(yawAngle)) + strafe * Math.cos(Math.toRadians(yawAngle));

        double frontLeftPower = (adjustedDrive + adjustedStrafe + rotate);
        double frontRightPower = (adjustedDrive - adjustedStrafe - rotate);
        double backLeftPower = (adjustedDrive - adjustedStrafe + rotate);
        double backRightPower = (adjustedDrive + adjustedStrafe - rotate);

        //Power
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("yaw",yawAngle);
    }

}
