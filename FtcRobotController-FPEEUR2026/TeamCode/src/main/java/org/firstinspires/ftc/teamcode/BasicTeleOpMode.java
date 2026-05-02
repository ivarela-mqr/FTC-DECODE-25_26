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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.subsystems.Zone;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp
public class BasicTeleOpMode extends OpMode {
    DriveTrain driveTrain;
    DcMotor intake;
    DcMotorEx transfer;
    IMU imu;
    YawPitchRollAngles orientation;
    double headingReset;
    double yawOffset = 0;
    Timer actualTimer,initTimer;
    boolean rumble = false;

    @Override
    public void init() {

        actualTimer = new Timer();
        initTimer = new Timer();
        driveTrain = new DriveTrain(hardwareMap);

        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        //tilt.Teleop(gamepad1);

        actualTimer.resetTimer();
        if(!rumble && timeElapsed() > 105){
            rumble = true;
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }
        if (gamepad1.left_trigger>0.1) {
            intake.setPower(1);
            transfer.setPower(1);
        }else {
            intake.setPower(0);
            transfer.setPower(0);
        }

        /*
        //telemetry.addData("Sensor2", intakeStateMachine.intake.colorSensor.distanceSensor.getDistance(DistanceUnit.CM));
        //telemetry.addData("Sensor2 bis", intakeStateMachine.intake.colorSensor.distanceSensor.getDistance(DistanceUnit.CM));
        //telemetry.addData("Sensor3", intakeStateMachine.intake.distanceSensor1.getDistance(DistanceUnit.CM));
        //telemetry.addData("Sensor1", intakeStateMachine.intake.distanceSensor2.getDistance(DistanceUnit.CM));
        //telemetry.addData("Intake state", intakeStateMachine.state);
        //telemetry.addData("Heading odometry", follower.getHeading());
        //telemetry.addData("SHooter", gamepad1.right_trigger);
        //telemetry.addData("Yaw imu", yawAngleShooter);
        //telemetry.addData("Is reseting", shooter.reset);
        //telemetry.addData("Offset", shooter.offset);
        //telemetry.addData("Power", shooter.rotorR.getPower());
        //telemetry.addData("Bumper", gamepad1.left_bumper);
        */

        //telemetry.addData("alliance",alliance);
        //telemetry.addData("YawAngle", yawAngle);
        //telemetry.addData("YawOffset", yawOffset);

        telemetry.update();
    }
    public int timeElapsed(){
        return (int)Math.abs(actualTimer.getElapsedTimeSeconds() - initTimer.getElapsedTimeSeconds());
    }

}
