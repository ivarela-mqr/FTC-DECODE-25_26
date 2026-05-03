package org.firstinspires.ftc.teamcode;



import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Tilt;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


@TeleOp
public class Italy26TeleOpMode extends OpMode {
    DriveTrain driveTrain;
    Shooter shooter;
    IntakeStateMachine intakeStateMachine;
    //Tilt tilt;

    Timer actualTimer,initTimer;
    boolean rumble = false;
    Constants.Alliance alliance = Constants.Alliance.BLUE;


    @Override
    public void init() {
        actualTimer = new Timer();
        initTimer = new Timer();
        driveTrain = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap, alliance, 1200);
        //tilt = new Tilt(hardwareMap);
        intakeStateMachine = new IntakeStateMachine(hardwareMap);
    }

    @Override
    public void start(){
        gamepad1.rumble(20);
        gamepad2.rumble(20);
        //tilt.resetTimer();
        initTimer.resetTimer();
        actualTimer.resetTimer();
        shooter.startTeleop();
    }


    @Override
    public void loop() {
        double yawAngle = driveTrain.TeleOp(gamepad1,telemetry);

        boolean isInShootZone = driveTrain.isInShootZone();


        intakeStateMachine.TeleOp((shooter.canShoot(gamepad1) && gamepad1.right_trigger > 0.1),
                                    gamepad1, gamepad2);
        shooter.TeleOp(gamepad1, gamepad2, telemetry, yawAngle, driveTrain.follower,
                                    intakeStateMachine.isFull(),isInShootZone);
        //tilt.Teleop(gamepad1);

        actualTimer.resetTimer();
        if(!rumble && timeElapsed() > 105){
            rumble = true;
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }

        telemetry.addData("Offset", shooter.turretOffset);
        telemetry.addData("Shooter Vel",shooter.shooter0.getVelocity());
        telemetry.addData("Pos hood",shooter.coverR.getPosition());
        telemetry.addData("Distance",driveTrain.follower.getPose().distanceFrom(shooter.goalPose));

        telemetry.update();

    }
    public int timeElapsed(){
        return (int)Math.abs(actualTimer.getElapsedTimeSeconds() - initTimer.getElapsedTimeSeconds());
    }

}
