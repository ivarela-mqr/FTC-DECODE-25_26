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
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "BLUE_near_partial", group = "Autonomous")
@Configurable
public class BLUE_near_partial extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private PathState lastPathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    IMU imu;
    YawPitchRollAngles orientation;
    ShootingStateMachine shootingStateMachine = new ShootingStateMachine();
    Timer stateTimer = new Timer();
    Timer actualTimer = new Timer();
    int numOpen = 0;
    int objNumOpen = 3;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(16, 114.5, Math.toRadians(180)));

        paths = new Paths(follower); // Build paths
        shootingStateMachine.init(hardwareMap, org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE,
                1475,IntakeStateMachineStates.FINAL,new Pose(54.5, 81.5));
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;


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
        shootingStateMachine.update(pose,telemetry,yawAngle,follower
                ,pathState != PathState.SHOOT_PRELOAD,true);
        autonomousPathUpdate();
        // Log values to Panels and Driver Station

        PoseStorage.update(follower.getPose(), org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE);
        panelsTelemetry.debug("Vel",shootingStateMachine.shooter.shooter0.getVelocity());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain goShootLoaded;
        public PathChain goTakeFirst;
        public PathChain goShootFirst;
        public PathChain goTakeSecond;
        public PathChain goTakeOpen;
        public PathChain goShootSecond;
        public PathChain finalPath;
        public PathChain goShootOpen;

        public Paths(Follower follower) {
            goShootLoaded = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(16, 114.5),
                                    new Pose(54.5, 81.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();

            goTakeSecond = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(54.5, 81.5),
                                    new Pose(48.855, 56.176),
                                    new Pose(6, 60)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goShootSecond = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(6, 60),
                                    new Pose(48.855, 56.176),
                                    new Pose(54.5, 81.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();

            goTakeOpen = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(54.5, 81.5),
                                    new Pose(33, 54),
                                    new Pose(10, 61)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(155))
                    .build();

            goShootOpen = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10, 61),
                                    new Pose(54.5, 81.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(155), Math.toRadians(230))
                    .build();
            goTakeFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(54.5, 81.5),
                                    new Pose(18,85)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierCurve(
                                    new Pose(18, 85),
                                    new Pose(35, 77),
                                    new Pose(10, 71)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goShootFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10, 71),
                                    new Pose(54.5, 81.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                    .build();
            finalPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(54.5, 81.5),
                                    new Pose(54.5, 110)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(270))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        actualTimer.resetTimer();
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(paths.goShootLoaded,1,true);
                shootingStateMachine.shooter.adjustCover(0.675);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && (!shootingStateMachine.isBusy() || numOpen == objNumOpen)) {
                     if (lastPathState == PathState.DRIVE_STARTPOS_SHOOT_POS){
                         follower.followPath(paths.goTakeSecond,1,true);
                         setPathState(PathState.TAKE_SECOND);
                    } else if (lastPathState == PathState.TAKE_SECOND) {
                         follower.followPath(paths.goTakeFirst,1,true);
                         setPathState(PathState.TAKE_FIRST);
                     } else if (lastPathState == PathState.TAKE_OPEN && numOpen < objNumOpen) {
                         follower.followPath(paths.goTakeOpen,1,true);
                         setPathState(PathState.TAKE_OPEN);
                     }else if(lastPathState == PathState.TAKE_OPEN && numOpen == objNumOpen) {
                         follower.followPath(paths.finalPath,1,true);
                         shootingStateMachine.shooter.adjustCover(0.525);
                         setPathState(PathState.END);
                    }else if(lastPathState == PathState.TAKE_FIRST){
                         follower.followPath(paths.goTakeOpen,1,true);
                         setPathState(PathState.TAKE_OPEN);
                     }
                }
                break;

            case TAKE_OPEN:
                if(!follower.isBusy() &&
                        (stateTimer.getElapsedTimeSeconds() > 3.5 ||
                                shootingStateMachine.intakeAutoStateMachine.state == IntakeStateMachineStates.FINAL
                                || isBlocked(3.5))){
                    follower.followPath(paths.goShootOpen,1,true);
                    numOpen ++;
                    setPathState(PathState.SHOOT_PRELOAD);
                    if(numOpen == objNumOpen)
                        shootingStateMachine.shooter.curTargetVelocity = 1400;
                }
                break;
            case TAKE_FIRST:
                if(!follower.isBusy() && (stateTimer.getElapsedTimeSeconds() > 2.75 || isBlocked(2.75))) {
                    follower.followPath(paths.goShootFirst,1,true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case TAKE_SECOND:
                if(!follower.isBusy()){
                    if (lastPathState == PathState.SHOOT_PRELOAD){
                        follower.followPath(paths.goShootSecond,1,true);
                        setPathState(PathState.SHOOT_PRELOAD);
                    }
                }
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
    public boolean isBlocked(double limitTime){
        return (stateTimer.getElapsedTimeSeconds() > limitTime)
                && (follower.getPreviousClosestPose().getPose().distanceFrom(follower.getPose()) < 0.05);
    }
}
