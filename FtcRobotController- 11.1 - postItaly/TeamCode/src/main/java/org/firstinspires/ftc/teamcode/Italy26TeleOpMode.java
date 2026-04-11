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
import org.firstinspires.ftc.teamcode.subsystems.Zone;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp
public class Italy26TeleOpMode extends OpMode {
    DriveTrain driveTrain;
    Shooter shooter;
    IntakeStateMachine intakeStateMachine;
    Tilt tilt;
    IMU imu;
    YawPitchRollAngles orientation;
    double headingReset;
    double yawOffset = 0;
    double yawOffsetShooter = 0;
    Follower follower;
    Timer actualTimer,initTimer;
    boolean rumble = false;
    Constants.Alliance alliance;
    Zone farZone,nearZone;
    double radius;

    @Override
    public void init() {
        alliance = PoseStorage.alliance;

        actualTimer = new Timer();
        initTimer = new Timer();
        driveTrain = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap, alliance, 1200);
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
        if (PoseStorage.currentPose != null) {
            follower.setPose(PoseStorage.currentPose);
            //follower.setStartingPose(PoseStorage.currentPose);
        }

        if (alliance == Constants.Alliance.BLUE){
            headingReset = 180;
        }else{
            headingReset = 0;
        }
        radius = Math.hypot(15.5,17.5)/2;
        farZone = new Zone(new Zone.Point(72,24), new Zone.Point(96,0),new Zone.Point(48,0),radius);
        nearZone = new Zone(new Zone.Point(72,72), new Zone.Point(0,144),new Zone.Point(144,144),radius);
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
        double yawAngleLimelight = orientation.getYaw(AngleUnit.DEGREES)
                                    - yawOffsetShooter;

        double rawYaw = Math.toDegrees(follower.getPose().getHeading());
        double yawAngle = rawYaw - yawOffset + headingReset;

        if (gamepad1.options) {
            yawOffset = rawYaw;
            follower.setPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    headingReset
            ));
            yawOffsetShooter = orientation.getYaw();
        }
        follower.update();
        if(isInShootZone()){
            gamepad1.rumble(100);
        }
        driveTrain.TeleOp(gamepad1,telemetry,yawAngle);
        intakeStateMachine.TeleOp((shooter.canShoot(gamepad1) && gamepad1.right_trigger > 0.1),
                                    gamepad1, gamepad2);
        shooter.TeleOp(gamepad1, gamepad2, telemetry, yawAngleLimelight,yawAngle,
                                    intakeStateMachine.isFull());
        tilt.Teleop(gamepad1);

        actualTimer.resetTimer();
        if(!rumble && timeElapsed() > 105){
            rumble = true;
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
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
        telemetry.addData("Pos encoder", shooter.encoder.getCurrentPosition());
        telemetry.addData("alliance",alliance);
        telemetry.addData("YawAngle", yawAngle);
        telemetry.addData("YawOffset", yawOffset);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());


        telemetry.update();
    }
    public int timeElapsed(){
        return (int)Math.abs(actualTimer.getElapsedTimeSeconds() - initTimer.getElapsedTimeSeconds());
    }

    boolean isInShootZone(){
        Pose pose = follower.getPose();
        if(farZone.isRobotInZone(pose))
            return true;
        return nearZone.isRobotInZone(pose);
    }
}
