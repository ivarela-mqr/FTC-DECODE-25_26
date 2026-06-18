package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.IntakeStateMachineStates;

public class ShootingStateMachine {
    Shooter shooter;
    Pose shootingPose;
    public enum States {
        INIT,
        LOADING,
        SHOOTING,
        INTAKING
    }
    public States state;
    public IntakeAutoStateMachine intakeAutoStateMachine = new IntakeAutoStateMachine();
    public boolean isInShootingPos = false;
    boolean canShoot = false;
    public Timer timer, actualTime;
    public Timer init;

    public void init(HardwareMap hardwareMap, Constants.Alliance alliance,
                     double currVel, IntakeStateMachineStates state, Pose shootingPose){
        shooter = new Shooter(hardwareMap, alliance,currVel);
        timer = new Timer();
        actualTime = new Timer();
        switchState(States.INIT);
        intakeAutoStateMachine.init(hardwareMap, state);
        this.shootingPose = shootingPose;
    }

    public void update(Pose pose, Telemetry telemetry, double yawAngleLimeLight, Follower follower, boolean isBusyFollower,
                       boolean insideTriangle){
        isInShootingPos = canShoot(pose);
        shooter.aim(yawAngleLimeLight,follower,insideTriangle);
        intakeAutoStateMachine.updateIntakeStateMachine(canShoot);
        actualTime.resetTimer();
        switch (state){
            case INIT:
                shooter.closeBlock();
                shooter.preload();
                if(isInShootingPos) {
                    shooter.preload();
                    switchState(States.LOADING);
                }
                break;
            case LOADING:
                if(shooter.isReady()
                    && Math.abs(timer.getElapsedTimeSeconds() - actualTime.getElapsedTimeSeconds())> 0.5) {
                    switchState(States.SHOOTING);
                    intakeAutoStateMachine.switchState(IntakeStateMachineStates.FINAL);
                } else
                    shooter.preload();
                break;
            case SHOOTING:
                shooter.openBlock();
                if(shooter.getBlockPos() <= 0.5){
                    canShoot = true;
                }
                if(Math.abs(timer.getElapsedTimeSeconds() - actualTime.getElapsedTimeSeconds())> 1){
                    canShoot = false;
                    switchState(States.INTAKING);
                    intakeAutoStateMachine.switchState(IntakeStateMachineStates.INIT);
                }
                break;
            case INTAKING:
                shooter.closeBlock();
                shooter.preload();
                if(!intakeAutoStateMachine.isBusy() )
                    switchState(States.INIT);
                break;
            default:
                break;
        }
    }
    public boolean canShoot(Pose pose) {
        double posTolerance = 15;
        double dx = pose.getX() - shootingPose.getX();
        double dy = pose.getY() - shootingPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance <= posTolerance;
    }
    public boolean isBusy(){
        return (state != States.INTAKING);
    }
    public void switchState(States state){
        this.state = state;
        timer.resetTimer();
    }
}
