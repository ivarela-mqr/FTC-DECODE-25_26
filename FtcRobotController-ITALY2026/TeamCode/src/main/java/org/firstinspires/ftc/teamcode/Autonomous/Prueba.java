package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "Prueba", group = "Autonomous")
@Configurable
public class Prueba extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private PathState lastPathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    IMU imu;
    YawPitchRollAngles orientation;
    public enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        TAKE_FIRST,
        TAKE_SECOND,
        TAKE_THIRD,
        TAKE_OPEN,
        END
    }
    ShootingStateMachine shootingStateMachine = new ShootingStateMachine();
    boolean shootsTriggered = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.305, 126.125, Math.toRadians(144)));

        paths = new Paths(follower); // Build paths


        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        shootingStateMachine.init(hardwareMap,
                org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE, new Pose(64, 85),
                new Pose(15,138),
                telemetry);

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
        shootingStateMachine.update(pose,telemetry,yawAngle, follower.isBusy());
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Last state",lastPathState);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter state",shootingStateMachine.state);
        panelsTelemetry.debug("Intake state",shootingStateMachine.intakeStateMachine.state);
        panelsTelemetry.debug("Is aiming auto", shootingStateMachine.shooter.autoAim);
        panelsTelemetry.debug("Rotors power", shootingStateMachine.shooter.rotorL.getPower());
        panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter0.getVelocity());
        panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter1.getVelocity());
        panelsTelemetry.debug("IMU", orientation.getYaw(AngleUnit.DEGREES));

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
        public PathChain goSHootOpen;

        public Paths(Follower follower) {
            goShotLoaded = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(21.305, 126.125),
                                    new Pose(53, 96)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(144))
                    .build();

            goTakeSecond1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53, 96),
                                    new Pose(53.000, 64.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                    .build();

            goTakeSecond2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53.000, 64.000),
                                    new Pose(17, 64.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goShotSecond = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(17, 64.000),
                                    new Pose(53, 93)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setGlobalDeceleration()
                    .setReversed()
                    .build();

            goOpen1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53, 93.000),
                                    new Pose(12, 58)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(170))
                    .build();


            goSHootOpen = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12, 58),
                                    new Pose(53, 96)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            goTakeFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53, 93),
                                    new Pose(16, 93)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setGlobalDeceleration()
                    .build();

            goShotFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(16, 93),
                                    new Pose(53, 96)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goTakeThird1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53, 96),
                                    new Pose(42.000, 40)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goTakeThird2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 40.000),
                                    new Pose(17, 40.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            goShotThird = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(17, 40.000),
                                    new Pose(53, 96)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setGlobalDeceleration()
                    .setReversed()
                    .build();
            finalPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53, 96),
                                    new Pose(41.929, 68.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }


    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                shootingStateMachine.shooter.adjustCover(0.3);
                follower.followPath(paths.goShotLoaded,0.9,true);
                lastPathState = pathState;
                pathState = PathState.SHOOT_PRELOAD;
                shootingStateMachine.shooter.initTimer();
                break;
            case SHOOT_PRELOAD:
                shootingStateMachine.shooter.autoAim = true;
                if(!shootingStateMachine.isBusy() && !follower.isBusy()) {
                    if(lastPathState == PathState.TAKE_FIRST) {
                        follower.followPath(paths.goTakeThird1,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.TAKE_THIRD;
                    } else if (lastPathState == PathState.TAKE_SECOND) {
                        follower.followPath(paths.goOpen1,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.TAKE_OPEN;
                    } else if (lastPathState == PathState.TAKE_THIRD) {
                        lastPathState = pathState;
                        pathState = PathState.END;
                    } else if(lastPathState == PathState.DRIVE_STARTPOS_SHOOT_POS){
                        follower.followPath(paths.goTakeSecond1,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.TAKE_SECOND;
                    } else if(lastPathState == PathState.TAKE_OPEN){
                        follower.followPath(paths.goTakeFirst,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.TAKE_FIRST;
                    }
                }
                break;
            case TAKE_FIRST:
                if(!follower.isBusy()) {
                    follower.followPath(paths.goShotFirst,0.9,true);
                    lastPathState = pathState;
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;
            case TAKE_OPEN:
                if(!follower.isBusy()&& !shootingStateMachine.intakeStateMachine.isBusy()) {

                    follower.followPath(paths.goSHootOpen,0.7,true);
                    lastPathState = pathState;
                    pathState = PathState.SHOOT_PRELOAD;

                }
                break;
            case TAKE_SECOND:
                shootingStateMachine.shooter.autoAim = false;
                shootingStateMachine.shooter.setPowerRotor(10);
                if(!follower.isBusy()) {
                    if (lastPathState == PathState.SHOOT_PRELOAD){
                        follower.followPath(paths.goTakeSecond2,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.TAKE_SECOND;
                    } else if (lastPathState == PathState.TAKE_SECOND) {
                        follower.followPath(paths.goShotSecond,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.SHOOT_PRELOAD;
                    }
                }
                break;

            case TAKE_THIRD:
                if(!follower.isBusy()) {
                    if (lastPathState == PathState.SHOOT_PRELOAD){
                        follower.followPath(paths.goTakeThird2,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.TAKE_THIRD;
                    } else if (lastPathState == PathState.TAKE_THIRD) {
                        follower.followPath(paths.goShotThird,0.9,true);
                        lastPathState = pathState;
                        pathState = PathState.SHOOT_PRELOAD;
                    }
                }
                break;
            case END:
                if(!follower.isBusy())
                    follower.followPath(paths.finalPath,true);
                break;
            default:
                break;
        }
    }
    public void setPathState(PathState state){
        //time.
    }
}
