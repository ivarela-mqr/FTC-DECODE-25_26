package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;


import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;


@TeleOp
public class BasicTeleOpMode extends OpMode {
    DriveTrain driveTrain;
    IntakeStateMachine intakeStateMachine;
    IMU imu;
    YawPitchRollAngles orientation;
    double yawOffset = 0;
    Timer actualTimer,initTimer;
    boolean rumble = false;

    @Override
    public void init() {

        actualTimer = new Timer();
        initTimer = new Timer();
        driveTrain = new DriveTrain(hardwareMap);

        intakeStateMachine = new IntakeStateMachine(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
    }

    @Override
    public void start(){
        gamepad1.rumble(20);
        gamepad2.rumble(20);

        initTimer.resetTimer();
        actualTimer.resetTimer();
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();

        double rawYaw = orientation.getYaw(AngleUnit.DEGREES);
        double yawAngle = rawYaw - yawOffset;

        if (gamepad1.options) {
            yawOffset = rawYaw;
        }

        driveTrain.TeleOp(gamepad1,telemetry,yawAngle);

        actualTimer.resetTimer();
        if(!rumble && timeElapsed() > 105){
            rumble = true;
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }
        intakeStateMachine.TeleOp(true,gamepad1,gamepad2);
        telemetry.addData("Sensor1", intakeStateMachine.intake.lightSensor1.isDetecting());
        telemetry.addData("Sensor2", intakeStateMachine.intake.lightSensor2.isPressed());
        telemetry.addData("Sensor3", intakeStateMachine.intake.lightSensor3.isDetecting());
        telemetry.addData("Intake state", intakeStateMachine.state);

        telemetry.update();
    }
    public int timeElapsed(){
        return (int)Math.abs(actualTimer.getElapsedTimeSeconds() - initTimer.getElapsedTimeSeconds());
    }

}
