package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class Italy26TeleOpMode extends OpMode {
    DriveTrain driveTrain;
    Shooter shooter;
    IntakeStateMachine intakeStateMachine;
    Tilt tilt;
    IMU imu;
    YawPitchRollAngles orientation;
    double yawOffset = 0;
    double yawOffsetShooter = 0;
    Follower follower;
    Timer actualTimer,initTimer;
    boolean rumble = false;
    @Override
    public void init() {
        actualTimer = new Timer();
        initTimer = new Timer();
        driveTrain = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap, Constants.Alliance.BLUE, 1200);
        tilt = new Tilt(hardwareMap);
        intakeStateMachine = new IntakeStateMachine(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        follower = createFollower(hardwareMap);
        follower.startTeleOpDrive(true);
        follower.setStartingPose(new Pose(72,72,0));
    }

    @Override
    public void start(){
        gamepad1.rumble(20);
        gamepad2.rumble(20);
        tilt.resetTimer();
        initTimer.resetTimer();
        actualTimer.resetTimer();
        shooter.startTeleop();
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        double yawAngleShooter = orientation.getYaw(AngleUnit.DEGREES)
                                    - yawOffsetShooter;
        double rawYaw = Math.toDegrees(follower.getHeading());
        double yawAngle = rawYaw - yawOffset;
        if (gamepad1.options) {
            yawOffset = rawYaw;
            follower.setPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    0
            ));
            yawOffsetShooter = orientation.getYaw();
        }
        follower.update();

        driveTrain.TeleOp(gamepad1,telemetry,yawAngle);
        intakeStateMachine.TeleOp((shooter.canShoot(gamepad1) && gamepad1.right_trigger > 0.1),
                                    gamepad1, gamepad2);
        shooter.TeleOp(gamepad1, gamepad2, telemetry, yawAngleShooter,
                                    intakeStateMachine.isFull());
        tilt.Teleop(gamepad1);

        actualTimer.resetTimer();
        if(!rumble && timeElapsed() > 105){
            rumble = true;
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }

        telemetry.addData("Sensor2", intakeStateMachine.intake.colorSensor.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Sensor2 bis", intakeStateMachine.intake.colorSensor.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Sensor3", intakeStateMachine.intake.distanceSensor1.getDistance(DistanceUnit.CM));
        telemetry.addData("Sensor1", intakeStateMachine.intake.distanceSensor2.getDistance(DistanceUnit.CM));
        telemetry.addData("Intake state", intakeStateMachine.state);
        telemetry.addData("Pos encoder", shooter.encoder.getCurrentPosition());
        telemetry.addData("Heading odometry", follower.getHeading());
        telemetry.addData("SHooter", gamepad1.right_trigger);
        telemetry.addData("Yaw imu", yawAngleShooter);

        telemetry.update();
    }
    public int timeElapsed(){
        return (int)Math.abs(actualTimer.getElapsedTimeSeconds() - initTimer.getElapsedTimeSeconds());
    }
}
