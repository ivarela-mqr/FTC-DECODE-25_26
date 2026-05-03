package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


public class DriveTrain {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    double rotVelFactor = 1;
    double driveVelFactor = 1;
    IMU imu;
    double yawOffset = 0;
    public YawPitchRollAngles orientation;
    double headingReset;

    public Follower follower;
    public Constants.Alliance alliance = Constants.Alliance.BLUE;
    private double radius;
    private Zone farZone,nearZone;
    private Pose resetPose;

    public DriveTrain(HardwareMap hardwareMap){
        //motors
        frontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = hardwareMap.get(DcMotor.class, "BackRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);

        //follower
        if (PoseStorage.currentPose != null) {
            alliance = PoseStorage.alliance;
        }

        follower = createFollower(hardwareMap);
        follower.startTeleOpDrive(true);
        if (PoseStorage.currentPose != null) {
            follower.setPose(PoseStorage.currentPose);
            //follower.setStartingPose(PoseStorage.currentPose);
            yawOffset = PoseStorage.currentPose.getHeading() > 0 ? 180 : -180;
        }else{
            follower.setPose(new Pose(8, 8, Math.toRadians(180)));
            yawOffset = 180;
        }
        if (alliance == Constants.Alliance.BLUE){
            headingReset = 180;
            resetPose = new Pose(
                    14.72,
                    78.30,
                    headingReset
            );
        }else{
            headingReset = 0;
            resetPose = new Pose(
                    129.28,
                    78.30,
                    headingReset
            );
            yawOffset = 0;
        }

        farZone = new Zone(new Zone.Point(72,24), new Zone.Point(96,0),new Zone.Point(48,0),radius);
        nearZone = new Zone(new Zone.Point(72,72), new Zone.Point(0,144),new Zone.Point(144,144),radius);
        radius = Math.hypot(15.5,17.5)/2;
    }

    public double TeleOp(Gamepad gamepad, Telemetry telemetry){
        orientation = imu.getRobotYawPitchRollAngles();

        double rawYaw = Math.toDegrees(follower.getHeading());
        double yawAngle = calcYawAngle(rawYaw);


        if (gamepad.options) {
            follower.setPose(resetPose);
        }
        follower.update();


        if (gamepad.left_stick_button){
            driveVelFactor = 0.25;
        }else{
            driveVelFactor = 1;
        }

        if (gamepad.right_stick_button){
            rotVelFactor = 0.25;
        }else{
            rotVelFactor = 1;
        }

        double drive = -gamepad.left_stick_y * driveVelFactor;
        double strafe = gamepad.left_stick_x * driveVelFactor;
        double rotate = gamepad.right_stick_x * rotVelFactor;


        // Small P controller
        double kP = 0.03; // todo tune this


        double headingRad = Math.toRadians(yawAngle);

        double rotX = drive * Math.cos(headingRad) - strafe * Math.sin(headingRad);
        double rotY = drive * Math.sin(headingRad) + strafe * Math.cos(headingRad);

        double frontLeftPower  = rotX + rotY + rotate;
        double frontRightPower = rotX - rotY - rotate;
        double backLeftPower   = rotX - rotY + rotate;
        double backRightPower  = rotX + rotY - rotate;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());

        //telemetry.addData("yaw",yawAngle);
        return yawAngle;
    }

    public boolean isInShootZone(){
        Pose pose = follower.getPose();
        if(farZone.isRobotInZone(pose))
            return true;
        return nearZone.isRobotInZone(pose);
    }


    private double calcYawAngle(double rawYaw){
        if (rawYaw > 0)
            return(rawYaw - yawOffset);
        else
            return(rawYaw + yawOffset);
    }

}
