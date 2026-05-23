package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.opencv.core.Mat;

public class Shooter {
    public  DcMotorEx shooter0, shooter1, encoder;
    public Servo coverL, coverR, block, rotorL, rotorR;
    //public LimeLight limeLight;
    int LEFT_LIMIT = -20000;
    int RIGHT_LIMIT = 20000;
    final double VELOCITY_FACTOR = 0.1;
    public double offset = 0, correctOffset = 0;
    double curTargetVelocity;
    public Timer init = new Timer();
    public boolean autoAim = true, teleOp = false;
    public Debouncer debouncer = new Debouncer(200);
    public Debouncer velDebouncer = new Debouncer(200);
    public Debouncer blockDebouncer = new Debouncer(500);
    PIDFCoefficients coefficients = new PIDFCoefficients(22, 0, 1.7, 15);
    private double lastValidOffset = 0;
    public boolean stop = false;
    private final ElapsedTime timer = new ElapsedTime();
    public double kP = 0.35;
    public double kD = 0.05;
    double lastError = 0;
    double lastTime = 0;
    Constants.Alliance alliance;
    public Pose goalPose;
    //public PoseCorrector poseCorrector;
    public boolean hold = true, reset = false;

    public Shooter (HardwareMap hardwareMap, Constants.Alliance alliance, double targetVel){
        shooter0 = hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1 = hardwareMap.get(DcMotorEx.class,"shooter1");
        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        rotorL = hardwareMap.get(Servo.class,"rotorL");
        rotorR = hardwareMap.get(Servo.class,"rotorR");
        coverL = hardwareMap.get(Servo.class,"coverL");
        coverR = hardwareMap.get(Servo.class,"coverR");
        block = hardwareMap.get(Servo.class, "block");
        shooter0.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, coefficients);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coverL.setDirection(Servo.Direction.REVERSE);
        this.alliance = alliance;
        //limeLight = new LimeLight(hardwareMap, alliance);
        if(alliance == Constants.Alliance.BLUE)
            goalPose = new Pose(3,130);
        else
            goalPose = new Pose(141,137);
        curTargetVelocity = targetVel;

        timer.reset();
    }
    public void resetTimer(){
        init.resetTimer();
    }

    public void aimWithOdometry(Follower follower){
        double allianceAngle = getTargetAngle(follower);//goal relative to field
        double targetAngle = 0; //target shooter relative to bot
        double relativeGoal = allianceAngle - Math.toDegrees(follower.getHeading()); //goal relative to bot
        if((relativeGoal > 330 || relativeGoal < 30) && teleOp) {
            setPowerRotor(0);
            return;
        }
        if(relativeGoal > 180){
            relativeGoal -= 360;
        }
        if(relativeGoal > 0){
            targetAngle = relativeGoal - 180;
        }else{
            targetAngle = relativeGoal + 180;
        }

        rotorR.setPosition(0.003030303*targetAngle + 0.5);
        rotorL.setPosition(0.003030303*targetAngle + 0.5);

    }
    public void aim(double yawLimelight, Follower pose, boolean isInShootingPos){
        if(isInShootingPos) {

        aimWithOdometry(pose);

        }else if(autoAim){
        }
    }

    //Trig utilities

    public int getTargetAngle(Follower follower){
        double allianceAngle = 0;
        if(alliance == Constants.Alliance.BLUE){
            allianceAngle = 180 - Math.toDegrees(Math.atan((goalPose.getY() - follower.getPose().getY())/(follower.getPose().getX() - goalPose.getX())));
        }else{
            allianceAngle = Math.toDegrees(Math.atan((goalPose.getY() - follower.getPose().getY())/( goalPose.getX() -follower.getPose().getX())));
        }
        return Math.toIntExact((long)allianceAngle);
    }
//BOOL states
    public boolean isReady(){
        return velocityOffset() < 50 && offset < 2;
    }
    public boolean isReady2(){
        return offset < 7.5 && offset > - 7.5;
    }
    public boolean canShoot(Gamepad gamepad){
        return (velocityOffset() < 50 && block.getPosition() < 0.1) || gamepad.left_trigger > 0.1;
    }



//STATES
    public void preload() {
        if (velocityOffset() > 100) {
            shooter0.setPower(1);
            shooter1.setPower(1);
        } else {
            shooter0.setVelocity(curTargetVelocity);
            shooter1.setVelocity(curTargetVelocity);
        }
    }
    public void stop() {
        shooter0.setPower(0);
        shooter1.setPower(0);
    }

    public void setPowerRotor(double power){
        int posR = encoder.getCurrentPosition();
        if (!reset && (posR <= LEFT_LIMIT && power > 0) ||
                (posR >= RIGHT_LIMIT && power < 0)) {
        }else{
            rotorL.setPosition(power);
            rotorR.setPosition(power);
        }
    }
//BLOCK usage
    public double getBlockPos() {
        return block.getPosition();
    }
    public void closeBlock() {
        block.setPosition(1);
    }
    public void openBlock(){
        block.setPosition(0);
    }

//COVER usage
    public void adjustCover(double dist){
        coverL.setPosition(dist);
        coverR.setPosition(dist);
    }
    public void correctCover(double direction){
        coverL.setPosition(coverL.getPosition()+(direction*0.05));
        coverR.setPosition(coverR.getPosition()+(direction*0.05));
    }
    public void adjustVelAndCover(Follower follower){
        double distance = getDistanceInches(follower);
        if(distance > 0 && teleOp && autoAim){
            double pos = 0.5042418 + 0.0003379807*distance - 0.00005000763*Math.pow(distance,2) + 1.815475e-7*Math.pow(distance,3);
            curTargetVelocity = 1967.261 - 25.41657*distance + 0.3601524*Math.pow(distance,2) - 0.001263507*Math.pow(distance,3);
            adjustCover(pos);
        }
    }

    public double velocityOffset(){
        return curTargetVelocity - Math.max(shooter1.getVelocity(), shooter0.getVelocity());
    }



    //Follower utilities
    public double getDistanceInches (Follower follower){
        return (follower.getPose().distanceFrom(goalPose));
    }
    public boolean isStable(Follower follower) {

        Vector vel = follower.getVelocity();

        double linearVel = Math.sqrt(Math.pow(vel.getXComponent(),2) + Math.pow(vel.getYComponent(),2));
        double angularVel = follower.getAngularVelocity();

        Vector accel = follower.getAcceleration();
        double linearAccel = Math.sqrt(Math.pow(accel.getXComponent(),2) + Math.pow(accel.getYComponent(),2));

        boolean lowLinear = linearVel < 3.0; // in/s
        boolean lowAngular = Math.abs(angularVel) < Math.toRadians(10);
        boolean lowAccel = linearAccel < 10.0; // in/s^2

        return lowLinear && lowAngular && lowAccel;
    }


    public void startTeleop(){
        teleOp = true;
        RIGHT_LIMIT = Constants.RIGHT_TELEOP_LIMIT;
        LEFT_LIMIT = Constants.LEFT_TELEOP_LIMIT;
    }
    public void TeleOp(Gamepad gamepad1, Gamepad gamepad2, Telemetry telemetry,
                       double yawAngleLimeLight, Follower follower , boolean isFull, boolean isInshootPos){
        if(!gamepad1.left_bumper)
            aim(yawAngleLimeLight,follower,isInshootPos);
        preload();
        adjustVelAndCover(follower);
        if(((isFull && isReady()) || gamepad1.left_trigger > 0.1) && gamepad1.right_trigger > 0.1)
            openBlock();
        else if(gamepad1.right_trigger < 0.5 && gamepad1.left_trigger < 0.5)
            closeBlock();

        if (gamepad2.share){
            autoAim = true;
            reset = false;
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(gamepad2.options)
            reset = true;
        if (gamepad2.dpad_left){
            alliance = Constants.Alliance.BLUE;
            //limeLight.switchAlliance(alliance);
            goalPose = new Pose(7,137);
        }else if(gamepad2.dpad_right){
            alliance = Constants.Alliance.RED;
            //limeLight.switchAlliance(alliance);
            goalPose = new Pose(137,137);
        }


        if (gamepad2.left_trigger > 0.1){
            autoAim = false;
            setPowerRotor(10);
        }else if (gamepad2.right_trigger > 0.1) {
            autoAim = false;
            setPowerRotor(-10);
        } else if (!autoAim){
            setPowerRotor(0);
            autoAim = false;
        }

        if(gamepad2.dpad_up && velDebouncer.isReady())
            curTargetVelocity += 50;
        if(gamepad2.dpad_down && velDebouncer.isReady())
            curTargetVelocity -= 50;
        if(gamepad2.left_bumper && debouncer.isReady())
            correctCover(-1);
        if(gamepad2.right_bumper && debouncer.isReady())
            correctCover(1);


        telemetry.addData("velocity shooter",shooter0.getVelocity());
        telemetry.addData("curTargetVelocity",curTargetVelocity);
        telemetry.addData("cover pos", coverR.getPosition());


        //Pose posLimelight = limeLight.getRawVisionPose();
        /*if(posLimelight != null){
            telemetry.addData("vision pos x", posLimelight.getX());
            telemetry.addData("vision pos y", posLimelight.getY());
        }
        }
        telemetry.addData("Stable", isStable(follower));
        telemetry.addData("shouldReset", poseCorrector.shouldReset(follower, getTurretAngle()));

         */



    }
}
