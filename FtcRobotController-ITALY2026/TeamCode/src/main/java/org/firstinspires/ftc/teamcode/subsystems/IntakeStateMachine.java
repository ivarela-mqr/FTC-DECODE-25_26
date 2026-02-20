package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Autonomous.ShootingStateMachine;

public class IntakeStateMachine {
    public Intake intake;
    public enum States{
        INIT,
        FIRST_ARTIFACT,
        SECOND_ARTIFACT,
        FINAL,
        SHOOTING
    }
    public States state;
    public Timer timer = new Timer();
    public Timer currTime = new Timer();

    public void init(HardwareMap hardwareMap){
        intake = new Intake(hardwareMap);
        state = States.FINAL;
    }
    public boolean isBusy() {
        return state != States.INIT && state != States.FIRST_ARTIFACT && state != States.SECOND_ARTIFACT;
    }
    public boolean isShooting() {
        return intake.numArtifactsIn() > 0;
    }

    public void updateIntakeStateMachine(boolean canShoot){
        currTime.resetTimer();
        switch (state){
            case INIT:
                intake.intakeFirstArtifact();
                if(intake.firstArtifactIn()
                        || Math.abs(currTime.getElapsedTimeSeconds() - timer.getElapsedTimeSeconds()) > 3){
                    switchState(States.FIRST_ARTIFACT);
                }
                break;
            case FIRST_ARTIFACT:
                intake.intakeNextArtifacts();
                if(intake.secondArtifactIn()
                        || Math.abs(currTime.getElapsedTimeSeconds() - timer.getElapsedTimeSeconds()) > 0.5){
                    switchState(States.SECOND_ARTIFACT);
                }
                break;
            case SECOND_ARTIFACT:
                intake.intakeNextArtifacts();;
                if(intake.thirdArtifactIn()
                        || Math.abs(currTime.getElapsedTimeSeconds() - timer.getElapsedTimeSeconds()) > 0.5){
                    switchState(States.FINAL);
                }
                break;
            case FINAL:
                intake.stopArtifacts();
                if(canShoot){switchState(States.SHOOTING);}
                break;
            case SHOOTING:
                intake.shootArtifacts();
                if(!canShoot
                        || Math.abs(currTime.getElapsedTimeSeconds() - timer.getElapsedTimeSeconds()) > 5){
                    switchState(States.INIT);
                }
                break;
        }
    }

    public void switchState(States state){
        this.state = state;
        timer.resetTimer();
    }
}
