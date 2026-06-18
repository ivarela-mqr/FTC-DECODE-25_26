package org.firstinspires.ftc.teamcode;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class FPEEUR26TeleOpMode extends OpMode {
    DriveTrain driveTrain;
    Shooter shooter;
    IntakeStateMachine intakeStateMachine;
    Timer actualTimer,initTimer;
    boolean rumble = false;

    @Override
    public void init() {
        actualTimer = new Timer();
        initTimer = new Timer();
        driveTrain = new DriveTrain(hardwareMap);
        shooter = new Shooter(hardwareMap, driveTrain.alliance, 1200);
        intakeStateMachine = new IntakeStateMachine(hardwareMap);
    }

    @Override
    public void start(){
        gamepad1.rumble(20);
        gamepad2.rumble(20);
        initTimer.resetTimer();
        actualTimer.resetTimer();
        shooter.startTeleop();
    }


    @Override
    public void loop() {
        double yawAngle = driveTrain.TeleOp(gamepad1,gamepad2,telemetry,Math.toRadians(shooter.getTurretAngle()),
                shooter.goalPose);

        //boolean isInShootZone = driveTrain.isInShootZone();

        intakeStateMachine.TeleOp((shooter.canShoot(gamepad1) && gamepad1.right_trigger > 0.1),
                                    gamepad1, gamepad2);
        shooter.TeleOp(gamepad1, gamepad2, telemetry, yawAngle, driveTrain.follower,
                                    intakeStateMachine.isFull(),true);

        actualTimer.resetTimer();
        if(!rumble && timeElapsed() > 105){
            rumble = true;
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }

        /*if(isInShootZone){
            gamepad1.rumble(1000);
        }*/

        //telemetry.addData("Shooter Vel",shooter.shooter0.getVelocity());
        //telemetry.addData("Pos hood",shooter.coverR.getPosition());
        //telemetry.addData("Angle shooter",shooter.getTurretAngle());
        //telemetry.addData("Target angle",shooter.getTurretAngle());
        //telemetry.addData("Angle robot",driveTrain.orientation.getYaw(AngleUnit.DEGREES));
        //telemetry.addData("Intake state",intakeStateMachine.state);
        telemetry.update();

    }
    public int timeElapsed(){
        return (int)Math.abs(actualTimer.getElapsedTimeSeconds() - initTimer.getElapsedTimeSeconds());
    }

}
