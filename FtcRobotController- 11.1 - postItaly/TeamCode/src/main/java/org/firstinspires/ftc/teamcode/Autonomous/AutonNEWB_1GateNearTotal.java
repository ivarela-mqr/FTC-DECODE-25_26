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

@Autonomous(name = "AutonNEWB_1GateNearTotal", group = "Autonomous")
@Configurable
public class AutonNEWB_1GateNearTotal extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private PathState lastPathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    IMU imu;
    YawPitchRollAngles orientation;
    ShootingStateMachine shootingStateMachine = new ShootingStateMachine();
    boolean shootsTriggered = false;
    int ticks = 0;
    Timer stateTimer = new Timer();
    Timer actualTimer = new Timer();
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.305, 126.125, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths


        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        shootingStateMachine.init(hardwareMap,
                org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE,1150, IntakeStateMachineStates.FINAL,
                new Pose(45, 96));

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
        panelsTelemetry.debug("Is aiming auto", shootingStateMachine.shooter.autoAim);
        panelsTelemetry.debug("Rotors power", shootingStateMachine.shooter.rotorL.getPower());
        panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter0.getVelocity());
        panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter1.getVelocity());
        panelsTelemetry.debug("Ticks", ticks);
        panelsTelemetry.debug("Is bussy", pathState != PathState.SHOOT_PRELOAD);

        //panelsTelemetry.debug("Can shoot",shootingStateMachine.canShoot(pose));
        //panelsTelemetry.debug("X", follower.getPose().getX());
        //panelsTelemetry.debug("Y", follower.getPose().getY());
        //panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain goShotLoaded;
        public PathChain goTakeFirst;
        public PathChain goShotFirst;
        public PathChain goTakeSecond1;
        public PathChain goTakeSecond2;
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
                                    new Pose(21.305, 126.125),
                                    new Pose(45,96)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(230))
                    .build();

            goTakeSecond1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(45,96),
                                    new Pose(45, 65)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(45, 65),
                                    new Pose(13, 65)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goOpen1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13, 65),
                                    new Pose(20, 65)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            goOpen2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(20, 65),
                                    new Pose(13, 73)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goShotSecond = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(13.000, 73.000),
                                    new Pose(39.988, 71.223),
                                    new Pose(45,96)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goTakeFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(45, 96),
                                    new Pose(45,90)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(45, 90),
                                    new Pose(18,90)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goShotFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(18, 90),
                                    new Pose(45,96)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();

            goTakeThird1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(45,96),
                                    new Pose(45, 43.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(45, 43.000),
                                    new Pose(17,43)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goShotThird = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(17, 43.000),
                                    new Pose(45,96)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();
            finalPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(45,96),
                                    new Pose(24, 70)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        actualTimer.resetTimer();
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                shootingStateMachine.shooter.adjustCover(0.4);
                follower.followPath(paths.goShotLoaded,1,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!shootingStateMachine.isBusy() && !follower.isBusy()) {
                    if(lastPathState == PathState.TAKE_FIRST) {
                        follower.followPath(paths.goTakeThird1,0.75,true);
                        setPathState(PathState.TAKE_THIRD);
                    } else if (lastPathState == PathState.OPEN_BLOCK) {
                        follower.followPath(paths.goTakeFirst,0.75,true);
                        setPathState(PathState.TAKE_FIRST);
                    } else if (lastPathState == PathState.TAKE_THIRD) {
                        follower.followPath(paths.finalPath,0.75,true);
                        setPathState(PathState.END);
                    } else if(lastPathState == PathState.DRIVE_STARTPOS_SHOOT_POS){
                        follower.followPath(paths.goTakeSecond1,0.75,true);
                        setPathState(PathState.TAKE_SECOND);
                    }
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
                        follower.followPath(paths.goOpen1,1,true);
                        setPathState(PathState.OPEN_BLOCK);
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
