package org.firstinspires.ftc.teamcode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Zone;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "Prueba15AutoBlue", group = "Autonomous")
@Configurable
public class Prueba15AutoBlue extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private PathState lastPathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    IMU imu;
    YawPitchRollAngles orientation;
    //ShootingStateMachine shootingStateMachine = new ShootingStateMachine();
    boolean shootsTriggered = false;
    int ticks = 0;
    Timer stateTimer = new Timer();
    Timer actualTimer = new Timer();
    int numOpen = 0;
    DcMotor intake;
    DcMotorEx transfer;

    Zone zone;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(15.000, 116.188, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths


        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu.initialize(parameters);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        zone = new Zone(new Zone.Point(72,72), new Zone.Point(0,144),new Zone.Point(144,144),
                Math.hypot(15.5,17.5)/2);
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        double yawAngle = orientation.getYaw(AngleUnit.DEGREES);
        follower.update(); // Update Pedro Pathing
        Pose pose = pathState == PathState.SHOOT_PRELOAD ? follower.getPose() : new Pose();

        autonomousPathUpdate(); // Update autonomous state machine
        // Log values to Panels and Driver Station
        //panelsTelemetry.debug("Last state",lastPathState);
        //panelsTelemetry.debug("Path State", pathState);
        //panelsTelemetry.debug("Shooter state",shootingStateMachine.state);
        //panelsTelemetry.debug("Intake state",shootingStateMachine.intakeAutoStateMachine.state);
        //panelsTelemetry.debug("Is aiming auto", shootingStateMachine.shooter.autoAim);
        //panelsTelemetry.debug("Rotors power", shootingStateMachine.shooter.rotorL.getPower());
        //panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter0.getVelocity());
        //panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter1.getVelocity());
        //panelsTelemetry.debug("Ticks", ticks);
        //panelsTelemetry.debug("Is bussy", pathState != PathState.SHOOT_PRELOAD);
        PoseStorage.update(follower.getPose(), org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE);
        //panelsTelemetry.debug("Can shoot",shootingStateMachine.canShoot(pose));
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain goShotLoaded;
        public PathChain goTakeFirst;
        public PathChain goShotFirst;
        public PathChain goTakeSecond1;
        public PathChain goTakeOpen;
        public PathChain goShotSecond;
        public PathChain goTakeThird1;
        public PathChain goTakeThird2;
        public PathChain goShotThird;
        public PathChain finalPath;
        public PathChain goOpen1;
        public PathChain goOpen2;
        public PathChain goSHootOpen;

        public Paths(Follower follower) {
            goShotLoaded = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(15.000, 116.188),
                                    new Pose(60.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();

            goTakeSecond1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(48.855, 56.176),
                                    new Pose(6.500, 62.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                    .build();

            goShotSecond = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(6.500, 62.000),
                                    new Pose(48.855, 56.176),
                                    new Pose(60.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();

            goTakeOpen = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(60.000, 84.000),
                                    new Pose(48.855, 56.176),
                                    new Pose(9.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(155))
                    .build();

            goSHootOpen = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(9.000, 60.000),
                                    new Pose(48.855, 56.176),
                                    new Pose(60.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(230))
                    .build();

        }
    }


    public void autonomousPathUpdate() {
        intake.setPower(1);
        transfer.setPower(1);
        actualTimer.resetTimer();
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(paths.goShotLoaded,1,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy()) {
                     if (lastPathState == PathState.TAKE_SECOND) {
                        follower.followPath(paths.goTakeOpen,1,true);
                        setPathState(PathState.TAKE_OPEN);
                    } else if (lastPathState == PathState.TAKE_OPEN && numOpen < 3) {
                        follower.followPath(paths.goTakeOpen,1,true);
                        setPathState(PathState.TAKE_OPEN);
                    } else if (lastPathState == PathState.TAKE_OPEN && numOpen == 3) {
                        //follower.followPath(paths.goTakeFirst,1,true);
                        setPathState(PathState.END);
                    } else if(lastPathState == PathState.DRIVE_STARTPOS_SHOOT_POS){
                        follower.followPath(paths.goTakeSecond1,1,true);
                        setPathState(PathState.TAKE_SECOND);
                    }
                }
                break;

            case TAKE_OPEN:
                if(!follower.isBusy() &&
                        Math.abs(actualTimer.getElapsedTimeSeconds() - stateTimer.getElapsedTimeSeconds()) > 3.5){
                    follower.followPath(paths.goSHootOpen,1,true);
                    numOpen ++;
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case TAKE_FIRST:
                if(!follower.isBusy()) {
                    follower.followPath(paths.goShotFirst,1,true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case TAKE_SECOND:
                if(!follower.isBusy()) {
                    if (lastPathState == PathState.SHOOT_PRELOAD){
                        follower.followPath(paths.goShotSecond,1,true);
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                }
                break;
            case TAKE_THIRD:
                if(!follower.isBusy()) {
                    if (lastPathState == PathState.SHOOT_PRELOAD) {
                        follower.followPath(paths.goShotThird,1,true);
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                }
                break;
            case OPEN_BLOCK:
                if(!follower.isBusy()){
                    if(lastPathState == PathState.TAKE_SECOND) {
                        follower.followPath(paths.goOpen2,0.5,true);
                        setPathState(PathState.OPEN_BLOCK);
                    }else if(lastPathState == PathState.OPEN_BLOCK &&
                            Math.abs(actualTimer.getElapsedTimeSeconds() - stateTimer.getElapsedTimeSeconds()) > 1.5){
                        follower.followPath(paths.goShotSecond,1, true);
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                }
                break;
            case END:
                break;
            default:
                break;
        }
    }
    public void setPathState(PathState state){
        stateTimer.resetTimer();
        lastPathState = pathState;
        pathState = state;
    }
}
