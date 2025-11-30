package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp

public class DriveBaseTeleOpMode extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;
    private YawPitchRollAngles orientation;
    private double yawOffset = 0;
    private double drive = 0;
    private double strafe = 0;
    private double rotate = 0;
    private boolean Reset = false;


    @Override
    public void runOpMode() {
// Inicializar los motores
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);



// Inicializar el IMU interno del Control Hub
        imu = hardwareMap.get(IMU.class, "imu");
// Configurar la orientación del IMU en el robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);


// Esperar el inicio
        waitForStart();

        this.gamepad1.rumble(1);
        this.gamepad2.rumble(1);

        while (opModeIsActive()) {


// Leer el ángulo de giro (yaw) del IMU interno
            drive = -gamepad1.left_stick_y; // Adelante/Atrás
            strafe = gamepad1.left_stick_x;
            rotate = -gamepad1.right_stick_x;
            orientation = imu.getRobotYawPitchRollAngles();
            double yawAngle = orientation.getYaw(AngleUnit.DEGREES) - yawOffset;
            double adjustedDrive = drive * Math.cos(Math.toRadians(yawAngle)) - strafe * Math.sin(Math.toRadians(yawAngle));
            double adjustedStrafe = drive * Math.sin(Math.toRadians(yawAngle)) + strafe * Math.cos(Math.toRadians(yawAngle));
            double frontLeftPower = (adjustedDrive + adjustedStrafe + rotate);
            double frontRightPower = (adjustedDrive - adjustedStrafe - rotate);
            double backLeftPower = (adjustedDrive - adjustedStrafe + rotate);
            double backRightPower = (adjustedDrive + adjustedStrafe - rotate);
            //Potencia
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            telemetry.addData("yaw",yawAngle);
        }
    }
}
