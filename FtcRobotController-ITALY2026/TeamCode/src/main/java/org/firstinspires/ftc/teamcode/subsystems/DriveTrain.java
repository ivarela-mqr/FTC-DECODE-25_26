package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class DriveTrain {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    double rotVelFactor = 1;
    double driveVelFactor = 1;
    public DriveTrain(HardwareMap hardwareMap){
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

    public void TeleOp(Gamepad gamepad, Telemetry telemetry, double yawAngle){
        if (gamepad.left_stick_button){
            driveVelFactor = 0.25;
        }else{
            driveVelFactor = 1;
        }

        if (gamepad.right_stick_button){
            rotVelFactor = 0.15;
        }else{
            rotVelFactor = 1;
        }

        double drive = -gamepad.left_stick_y * driveVelFactor;
        double strafe = gamepad.left_stick_x * driveVelFactor;
        double rotate = gamepad.right_stick_x * rotVelFactor;

        double headingRad = Math.toRadians(yawAngle);

        double rotX = drive * Math.cos(headingRad) - strafe * Math.sin(headingRad);
        double rotY = drive * Math.sin(headingRad) + strafe * Math.cos(headingRad);

        double frontLeftPower  = rotX + rotY + rotate;
        double frontRightPower = rotX - rotY - rotate;
        double backLeftPower   = rotX - rotY + rotate;
        double backRightPower  = rotX + rotY - rotate;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("yaw",yawAngle);
    }
}
