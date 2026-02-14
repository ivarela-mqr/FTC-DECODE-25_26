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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name = "Solo Autonomous", group = "Autonomous")
@Configurable
public class SoloAutonomous extends OpMode {
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
        END
    }
    ShootingStateMachine shootingStateMachine = new ShootingStateMachine();
    boolean shootsTriggered = false;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.146, 123.261, Math.toRadians(142)));

        paths = new Paths(follower); // Build paths


        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        shootingStateMachine.init(hardwareMap, org.firstinspires.ftc.teamcode.util.Constants.Alliance.BLUE);

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
        shootingStateMachine.update(pose,telemetry,yawAngle);
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("Shooter state",shootingStateMachine.state);
        panelsTelemetry.debug("Can shoot",shootingStateMachine.canShoot(pose));
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());

        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.146, 123.261),

                                    new Pose(72.510, 84.611)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(142))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.510, 84.611),

                                    new Pose(13.497, 84.076)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(160), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.497, 84.076),

                                    new Pose(71.777, 84.382)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(71.777, 84.382),

                                    new Pose(52.917, 59.325)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(52.917, 59.325),

                                    new Pose(13.822, 59.605)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.822, 59.605),

                                    new Pose(72.357, 84.344)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(72.357, 84.344),

                                    new Pose(53.478, 36.159)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(53.478, 36.159),

                                    new Pose(13.032, 35.764)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.032, 35.764),

                                    new Pose(72.529, 84.535)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                    .build();
        }
    }



    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine

        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(paths.Path1);
                pathState = PathState.SHOOT_PRELOAD;
                break;
            case SHOOT_PRELOAD:
                if(!shootingStateMachine.isBusy()){
                    if(lastPathState == PathState.TAKE_FIRST)
                        pathState = PathState.END;
                    else {
                        follower.followPath(paths.Path3);
                        lastPathState = pathState;
                        pathState = PathState.TAKE_FIRST;
                    }
                }
                break;
            case TAKE_FIRST:
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path4);
                    lastPathState = pathState;
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;
            case END:

                break;
            default:
                break;
        }
    }
    public void setPathState(PathState state){
        //time.
    }
}
