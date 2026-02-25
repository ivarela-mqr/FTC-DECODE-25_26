package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class Italy26TeleOpMode extends OpMode {
    DriveTrain driveTrain;
    Intake intake;
    Shooter shooter;
    Tilt tilt;
    IMU imu;
    YawPitchRollAngles orientation;
    double yawOffset = 0;
    IntakeStateMachine intakeStateMachine = new IntakeStateMachine();
    Follower follower;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, Constants.Alliance.BLUE, 1200, 30); //todo ch. alliance from auton
        tilt = new Tilt(hardwareMap);
        intakeStateMachine.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive(true);
        follower.setStartingPose(new Pose(72,72,0));
    }

    @Override
    public void start(){
        gamepad1.rumble(20);
        gamepad2.rumble(20);
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();

        double yawAngleShooter = orientation.getYaw(AngleUnit.DEGREES) - yawOffset;
        if(gamepad1.options){
            //yawOffset = orientation.getYaw();
        }
        double rawYaw = Math.toDegrees(follower.getHeading());
        double yawAngle = rawYaw - yawOffset;

        if (gamepad1.options) {
            yawOffset = rawYaw;
        }
        if (gamepad1.options) {
            follower.setPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    0
            ));
        }
        follower.update();
        //switch Alliance
        if (gamepad2.dpad_left){
            shooter.limeLight.switchAlliance(Constants.Alliance.BLUE);
        }else if(gamepad2.dpad_right){
            shooter.limeLight.switchAlliance(Constants.Alliance.RED);
        }

        driveTrain.TeleOp(gamepad1,telemetry,yawAngle);
        intakeStateMachine.updateIntakeStateMachine((shooter.canShoot() && gamepad1.right_trigger > 0.1)
                ,gamepad1,gamepad2);
        shooter.TeleOp(gamepad1, gamepad2, telemetry, yawAngleShooter, intakeStateMachine.isFull());
        tilt.Teleop(gamepad1);
        telemetry.addData("Sensor1", intakeStateMachine.intake.firstArtifactIn());
        telemetry.addData("Sensor2", intakeStateMachine.intake.distanceSensor1.getDistance(DistanceUnit.CM));
        telemetry.addData("Sensor3", intakeStateMachine.intake.distanceSensor2.getDistance(DistanceUnit.CM));
        telemetry.addData("Intake state", intakeStateMachine.state);
        telemetry.addData("Is shooting", intake.getIsShooting());
        telemetry.addData("Pos encoder", shooter.encoder.getCurrentPosition());
        telemetry.addData("Heading odometry", follower.getHeading());
        telemetry.addData("Yaw imu", yawAngleShooter);

        telemetry.update();
    }
}
