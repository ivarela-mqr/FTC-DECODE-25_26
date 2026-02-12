package org.firstinspires.ftc.teamcode;
import com.pedropathing.paths.Path;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private PathState pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    public enum PathState{
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        GO_SHOOT,
        DEFAULT,
        END
    }
    DcMotorEx shooter1, shooter2;
    DcMotorEx  intake, transfer;
    Servo coverL, coverR, block;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;

        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients coeficients = new PIDFCoefficients(1,0,0,15.3);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);

        coverL = hardwareMap.get(Servo.class,"coverL");
        coverR = hardwareMap.get(Servo.class,"coverR");
        block = hardwareMap.get(Servo.class,"block");
        coverL.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
        coverL.setPosition(0.6);
        coverR.setPosition(0.6);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(56.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 36.000),

                                    new Pose(8, 35.242)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8, 35.242),

                                    new Pose(70.631, 73.223)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(70.631, 73.223),

                                    new Pose(53.598726114649686, 90)
                            )
                    ).setTangentHeadingInterpolation()

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
                if(!follower.isBusy())
                {
                    follower.followPath(paths.Path2);
                    pathState = PathState.GO_SHOOT;
                    intake.setPower(1);
                    transfer.setPower(1);
                }

                break;
            case GO_SHOOT:
                if(!follower.isBusy())
                {
                    follower.followPath(paths.Path3);
                    intake.setPower(0.25);
                    transfer.setPower(0);

                        shooter1.setVelocity(1200);
                        shooter2.setVelocity(1200);

                    block.setPosition(0.5);
                    pathState = PathState.DEFAULT;
                }

            case DEFAULT:
                if(!follower.isBusy()){
                    follower.followPath(paths.Path4);

                    pathState = PathState.END;
                }

                break;
            case END:
                if(!follower.isBusy())
                {
                    if(block.getPosition()!=0.5)
                        block.setPosition(0.5);
                    else {
                        intake.setPower(1);
                        transfer.setPower(1);
                    }
                }
            default:
                break;



        }
    }
    public void setPathState(PathState state){
        //time.
    }
}
