package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutonB_NearPreTotal extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;
        public PathChain Path13;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(27.214, 131.413),
                                    new Pose(81.153, 83.689)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(142))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(81.153, 83.689),
                                    new Pose(14.500, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.500, 84.000),
                                    new Pose(64.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(64.000, 84.000),
                                    new Pose(53.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path5 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(53.000, 60.000),
                                    new Pose(14.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path6 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.000, 60.000),
                                    new Pose(42.000, 117.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 117.000),
                                    new Pose(42.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(42.000, 36.000),
                                    new Pose(14.000, 35.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path9 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(14.000, 35.000),
                                    new Pose(73.000, 10.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path10 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(73.000, 10.000),
                                    new Pose(12.500, 34.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                    .build();

            Path11 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(12.500, 34.400),
                                    new Pose(13.000, 60.400)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(150))
                    .build();

            Path12 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(13.000, 60.400),
                                    new Pose(73.000, 9.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                    .build();

            Path13 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(73.000, 9.000),
                                    new Pose(10.000, 25.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return 0;
    }
}
