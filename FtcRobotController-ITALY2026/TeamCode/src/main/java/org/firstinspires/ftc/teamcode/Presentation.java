package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class Presentation extends OpMode {
    DriveTrain driveTrain;
    Shooter shooter;
    Intake intake;
    Tilt tilt;
    Timer timer,actualTimer;
    LimeLight limelight;
    IMU imu;
    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap, Constants.Alliance.BLUE, 1200);
        tilt = new Tilt(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new LimeLight(hardwareMap, Constants.Alliance.BLUE);
        timer = new Timer();
        actualTimer = new Timer();
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
    }
    @Override
    public void start(){
        timer.resetTimer();
        actualTimer.resetTimer();
    }
    @Override
    public void loop() {
        shooter.aimWithLimelight(imu.getRobotYawPitchRollAngles().getYaw());
        actualTimer.resetTimer();
        if(gamepad1.x){
            timer.resetTimer();
            shooter.preload();
        }else if(timeElapsed() > 1){
            shooter.stop();
        }

        if(gamepad1.dpadUpWasPressed()){
            timer.resetTimer();
            intake.demo();
        }else if(timeElapsed() > 1){
            intake.stopArtifacts();
        }

        if(gamepad1.dpadDownWasPressed()){
            timer.resetTimer();
            tilt.up();
        }else if(timeElapsed() > 2.5)
            tilt.stop();

    }
    public int timeElapsed(){
        return (int)Math.abs(timer.getElapsedTimeSeconds() - actualTimer.getElapsedTimeSeconds());
    }
}
