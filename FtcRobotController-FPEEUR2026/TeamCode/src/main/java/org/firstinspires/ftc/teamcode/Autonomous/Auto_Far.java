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

@Autonomous(name = "Auto_Far", group = "Autonomous")
@Configurable
public class Auto_Far extends OpMode {
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
    int numOpen = 0;
    int objNumOpen = 3;
    IntakeAutoStateMachine intakeAutoStateMachine = new IntakeAutoStateMachine();

    Zone zone;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56.306, 7.389, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths
        shootingStateMachine.init(hardwareMap, org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE,
                1900,IntakeStateMachineStates.FINAL,new Pose(53, 15));
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
        public PathChain goTakeThird;
        public PathChain goShootSecond;
        public PathChain finalPath;
        public PathChain goShootThird;
        public PathChain goOpen1;

        public Paths(Follower follower) {
            goShootLoaded = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.306, 7.389),
                                    new Pose(53.000, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))

                    .build();

            goTakeSecond = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.000, 10),
                                    new Pose(9.000, 6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            goShootSecond = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9, 6),
                                    new Pose(53.000, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            goTakeThird = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.000, 10),
                                    new Pose(59.516, 38.369),
                                    new Pose(10, 45)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            goShootThird = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(10, 45),
                                    new Pose(53.000, 10)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            goTakeFirst = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(53.000, 10),
                                    new Pose(8.000, 50.000),
                                    new Pose(9.000, 8)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            goShootFirst = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.000, 8),
                                    new Pose(53.000, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                    .build();
            finalPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53,80),
                                    new Pose(25, 66)
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
                follower.followPath(paths.goShootLoaded,1,true);
                shootingStateMachine.shooter.adjustCover(0.75);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && !shootingStateMachine.isBusy()) {
                    if (lastPathState == PathState.DRIVE_STARTPOS_SHOOT_POS){
                        follower.followPath(paths.goTakeSecond,0.7,true);
                        setPathState(PathState.TAKE_SECOND);
                    } else if (lastPathState == PathState.TAKE_SECOND) {
                        follower.followPath(paths.goTakeThird,0.8,true);
                        setPathState(PathState.TAKE_THIRD);
                    } else if (lastPathState == PathState.TAKE_OPEN && numOpen < objNumOpen) {
                        follower.followPath(paths.goTakeThird,0.7,true);
                        setPathState(PathState.TAKE_OPEN);
                    } else if(lastPathState == PathState.TAKE_OPEN && numOpen == objNumOpen) {
                        follower.followPath(paths.goTakeFirst,1,true);
                        setPathState(PathState.TAKE_FIRST);
                    }else if(lastPathState == PathState.TAKE_FIRST){
                        follower.followPath(paths.goTakeFirst,1,true);
                        setPathState(PathState.TAKE_FIRST);
                    }else if(lastPathState == PathState.TAKE_THIRD){
                        follower.followPath(paths.goTakeFirst,0.7,true);
                        setPathState(PathState.TAKE_FIRST);
                    }
                }
                break;

            case TAKE_OPEN:
                if(!follower.isBusy() && stateTimer.getElapsedTimeSeconds() > 3.5){
                    follower.followPath(paths.goShootThird,1,true);
                    numOpen ++;
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case TAKE_FIRST:
                if(!follower.isBusy()) {
                    follower.followPath(paths.goShootFirst,1,true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }

            case TAKE_THIRD:
                if(!follower.isBusy()) {
                    follower.followPath(paths.goShootThird,1,true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case TAKE_SECOND:
                if(!follower.isBusy()) {
                    if (lastPathState == PathState.SHOOT_PRELOAD && stateTimer.getElapsedTimeSeconds() > 2){
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
}
