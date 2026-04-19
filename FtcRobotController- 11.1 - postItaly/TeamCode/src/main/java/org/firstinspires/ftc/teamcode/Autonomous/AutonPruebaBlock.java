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

@Autonomous(name = "AutonPruebaBlock", group = "Autonomous")
@Configurable
public class AutonPruebaBlock extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private PathState lastPathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    IMU imu;
    YawPitchRollAngles orientation;
    int ticks = 0;
    Timer stateTimer = new Timer();
    Timer actualTimer = new Timer();
    Zone zone;

    int limitTime;



    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        paths = new Paths(follower); // Build paths


        pathState = PathState.PATH1;


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
        Pose pose = pathState == PathState.PATH1 ? follower.getPose() : new Pose();
        autonomousPathUpdate();


        // Log values to Panels and Driver Station
        //panelsTelemetry.debug("Last state",lastPathState);
        //panelsTelemetry.debug("Path State", pathState);
        //panelsTelemetry.debug("Shooter state",shootingStateMachine.state);
        //panelsTelemetry.debug("Intake state",shootingStateMachine.intakeAutoStateMachine.state);
        //panelsTelemetry.debug("Is aiming auto", shootingStateMachine.shooter.autoAim);
        //panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter0.getVelocity());
        //panelsTelemetry.debug("Shooter velocity", shootingStateMachine.shooter.shooter1.getVelocity());
        //panelsTelemetry.debug("Ticks", ticks);
        //panelsTelemetry.debug("Is bussy", pathState != PathState.SHOOT_PRELOAD);
        PoseStorage.update(follower.getPose(), org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE);
        //panelsTelemetry.debug("Can shoot",shootingStateMachine.canShoot(pose));
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getHeading()));
        panelsTelemetry.debug("Previous pose X", follower.getPreviousClosestPose().getPose().getX());
        panelsTelemetry.debug("Previous pose Y", follower.getPreviousClosestPose().getPose().getY());

        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain forward1;
        public PathChain backward1;
        public PathChain forward2;
        public PathChain backward2;


        public Paths(Follower follower) {
            forward1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(0, 0),
                                    new Pose(45,0)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            backward1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(45,0),
                                    new Pose(0, 0)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            forward2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(0, 0),
                                    new Pose(0, 45)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();
            backward2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(0, 45),
                                    new Pose(0, 0)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();


        }
    }


    public void autonomousPathUpdate() {
        actualTimer.resetTimer();
        switch (pathState){
            case PATH1:
                follower.followPath(paths.forward1,0.25,true);
                setPathState(PathState.PATH2);
                break;

            case PATH2:
                if(!follower.isBusy() || isBlocked(2)) {

                    follower.followPath(paths.backward1,0.25,true);
                    setPathState(PathState.PATH3);
                }
                break;
            case PATH3:
                if(!follower.isBusy() || isBlocked(2)) {
                    follower.followPath(paths.forward2,0.25,true);
                    setPathState(PathState.PATH4);
                }
                break;
            case PATH4:
                if(!follower.isBusy() || isBlocked(2)) {
                    follower.followPath(paths.backward2,0.25,true);
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
    public boolean isBlocked(int limitTime){
        return (stateTimer.getElapsedTimeSeconds() > limitTime)
                && (follower.getPreviousClosestPose().getPose().distanceFrom(follower.getPose()) < 0.05);
    }

}
