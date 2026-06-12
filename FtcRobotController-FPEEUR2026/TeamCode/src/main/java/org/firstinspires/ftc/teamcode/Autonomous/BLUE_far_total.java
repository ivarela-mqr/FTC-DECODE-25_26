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
import org.firstinspires.ftc.teamcode.subsystems.Zone;
import org.firstinspires.ftc.teamcode.util.IntakeStateMachineStates;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@Autonomous(name = "BLUE_far", group = "Autonomous")
@Configurable
public class BLUE_far_total extends OpMode {
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
    boolean thirdTaken = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.306, 7.389, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths
        shootingStateMachine.init(hardwareMap, org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE,
                1850,IntakeStateMachineStates.FINAL,new Pose(53, 15));
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
        public PathChain goTakeGate;
        public PathChain goShootGate;
        public PathChain goTakeBase;
        public PathChain goTakeThird;
        public PathChain goShootBase;
        public PathChain goShootThird;

        public Paths(Follower follower) {
            goShootLoaded = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.306, 7.389),
                                    new Pose(53.000, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                    .build();

            goTakeBase = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.000, 10),
                                    new Pose(9.000, 6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            goShootBase = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9, 6),
                                    new Pose(53.000, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            goTakeThird = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.000, 10),
                                    new Pose(40, 35)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .addPath(
                            new BezierLine(
                                    new Pose(40, 35),
                                    new Pose(13, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            goShootThird = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13, 35),
                                    new Pose(53.000, 10)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            goTakeGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.000, 10),
                                    new Pose(6, 40)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            goShootGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6, 40),
                                    new Pose(53.000, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        actualTimer.resetTimer();
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(paths.goShootLoaded,1,true);
                shootingStateMachine.shooter.adjustCover(0.75);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && !shootingStateMachine.isBusy()) {
                    if (lastPathState == PathState.DRIVE_STARTPOS_SHOOT_POS ||
                            lastPathState == PathState.TAKE_GATE || lastPathState == PathState.TAKE_THIRD){
                        follower.followPath(paths.goTakeBase,0.9,true);
                        setPathState(PathState.TAKE_BASE);
                    } else if (lastPathState == PathState.TAKE_BASE && !thirdTaken) {
                        follower.followPath(paths.goTakeThird,0.9,true);
                        setPathState(PathState.TAKE_THIRD);
                        thirdTaken = true;
                    }else if(lastPathState == PathState.TAKE_BASE){
                        follower.followPath(paths.goTakeGate,0.9,true);
                        setPathState(PathState.TAKE_GATE);
                    }
                }
                break;

            case TAKE_GATE:
                if(!follower.isBusy()) {
                    follower.followPath(paths.goShootGate,1,true);
                    shootingStateMachine.intakeAutoStateMachine.switchState(IntakeStateMachineStates.FINAL);
                    setPathState(PathState.SHOOT_PRELOAD);
                }

            case TAKE_THIRD:
                if(!follower.isBusy()) {
                    follower.followPath(paths.goShootThird,1,true);
                    shootingStateMachine.intakeAutoStateMachine.switchState(IntakeStateMachineStates.FINAL);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case TAKE_BASE:
                if(!follower.isBusy()) {
                    if (lastPathState == PathState.SHOOT_PRELOAD && stateTimer.getElapsedTimeSeconds() > 2){
                        follower.followPath(paths.goShootBase,1,true);
                        shootingStateMachine.intakeAutoStateMachine.switchState(IntakeStateMachineStates.FINAL);
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
}
