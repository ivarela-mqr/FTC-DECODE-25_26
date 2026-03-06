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
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.IntakeStateMachineStates;

@Autonomous(name = "RED_0GateFarPreload", group = "Autonomous")
@Configurable
public class AutonR_0GateFarPreload extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private PathState lastPathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    IMU imu;
    YawPitchRollAngles orientation;
    ShootingStateMachine shootingStateMachine = new ShootingStateMachine();
    int ticks = 0;
    Timer stateTimer = new Timer();
    Timer actualTimer = new Timer();
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(99, 7, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths


        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        shootingStateMachine.init(hardwareMap,
                org.firstinspires.ftc.teamcode.util.Constants.Alliance.RED,1400, IntakeStateMachineStates.FINAL,
                new Pose(81,10));

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        double yawAngle = orientation.getYaw(AngleUnit.DEGREES);
        follower.update(); // Update Pedro Pathing
        Pose pose = pathState == PathState.SHOOT_PRELOAD ? follower.getPose() : new Pose();
        shootingStateMachine.update(pose,telemetry,yawAngle, pathState != PathState.SHOOT_PRELOAD);
        autonomousPathUpdate(); // Update autonomous state machine
        ticks ++;
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Last state",lastPathState);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter state",shootingStateMachine.state);
        panelsTelemetry.debug("Intake state",shootingStateMachine.intakeAutoStateMachine.state);
        panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter0.getVelocity());

        //panelsTelemetry.debug("Can shoot",shootingStateMachine.canShoot(pose));
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        //panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain goShotLoaded;
        public PathChain goTakeBase1;
        public PathChain goTakeBase2;
        public PathChain goShotBase;
        public PathChain goTakeThird1;
        public PathChain goTakeThird2;
        public PathChain goShotThird;
        public PathChain finalPath;

        public Paths(Follower follower) {
            goShotLoaded = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(99, 7),
                                    new Pose(84, 16)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(36))
                    .build();

            goTakeThird1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(84, 16),
                                    new Pose(102, 20)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                    .build();

            goTakeThird2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42, 40),
                                    new Pose(10, 40)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
            goShotThird = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10, 40),
                                    new Pose(60, 8)
                            )
                    )
                    .setGlobalDeceleration()
                    //.setReversed()
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();

            goTakeBase1 = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    new Pose(60, 8),
                                    new Pose(30, 22)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(245)
                    )

                    .addPath(
                            new BezierLine(
                                    new Pose(30, 22),
                                    new Pose(3.5, 37)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(245))

                    .setGlobalDeceleration()
                    .build();

            goTakeBase2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(3.5, 37),
                                    new Pose(1, 9)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(245), Math.toRadians(260))
                    .build();

            goShotBase = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(1, 9),
                                    new Pose(63, 16)
                            )
                    )
                    //.setTangentHeadingInterpolation()
                    .setGlobalDeceleration()
                    .setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(90))
                    .build();

            finalPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(63, 16),
                                    new Pose(40, 40)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        actualTimer.resetTimer();
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                shootingStateMachine.shooter.adjustCover(0.4);
                shootingStateMachine.shooter.correctOffset = - 5;
                follower.followPath(paths.goShotLoaded,0.5,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!shootingStateMachine.isBusy() && !follower.isBusy()) {
                    if (lastPathState == PathState.TAKE_THIRD) {
                        follower.followPath(paths.goTakeBase1, 0.75, true);
                        setPathState(PathState.TAKE_BASE);

                    } else if (lastPathState == PathState.TAKE_BASE) {
                        follower.followPath(paths.finalPath, 1,true);
                        setPathState(PathState.END);
                    } else if(lastPathState == PathState.DRIVE_STARTPOS_SHOOT_POS){
                        follower.followPath(paths.goTakeThird1,0.75,true);
                        setPathState(PathState.END);
                    }
                }
                break;

            case TAKE_THIRD:
                if(!follower.isBusy()) {
                    if (lastPathState == PathState.SHOOT_PRELOAD){
                        follower.followPath(paths.goTakeThird2,0.5,true);
                        setPathState(PathState.TAKE_THIRD);
                    } else if (lastPathState == PathState.TAKE_THIRD) {
                        follower.followPath(paths.goShotThird,0.65,true);
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                }
                break;

            case TAKE_BASE:
                if(!follower.isBusy()){
                    if(lastPathState == PathState.SHOOT_PRELOAD) {
                        follower.followPath(paths.goTakeBase2,0.5,true);
                        setPathState(PathState.TAKE_BASE);
                    }else if(lastPathState == PathState.TAKE_BASE &&
                            Math.abs(actualTimer.getElapsedTimeSeconds() - stateTimer.getElapsedTimeSeconds()) > 1){
                        follower.followPath(paths.goShotBase,0.75, true);
                        setPathState(PathState.SHOOT_PRELOAD);
                    }

                }
                break;
            case END:
                shootingStateMachine.shooter.autoAim = false;
                //shootingStateMachine.shooter.resetRotorPosition();
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
