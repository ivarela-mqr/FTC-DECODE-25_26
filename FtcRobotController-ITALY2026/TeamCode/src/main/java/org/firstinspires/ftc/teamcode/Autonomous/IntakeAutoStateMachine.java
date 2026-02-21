package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.IntakeStateMachineStates;

public class IntakeAutoStateMachine {
    public Intake intake;
    public IntakeStateMachineStates state;
    public Timer timer = new Timer();
    public Timer currTime = new Timer();

    public void init(HardwareMap hardwareMap){
        intake = new Intake(hardwareMap);
        state = IntakeStateMachineStates.FINAL;
    }
    public boolean isBusy() {
        return state != IntakeStateMachineStates.INIT && state != IntakeStateMachineStates.FIRST_ARTIFACT && state != IntakeStateMachineStates.SECOND_ARTIFACT;
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
                    switchState(IntakeStateMachineStates.FIRST_ARTIFACT);
                }
                break;
            case FIRST_ARTIFACT:
                intake.intakeNextArtifacts();
                if(intake.secondArtifactIn()
                        || Math.abs(currTime.getElapsedTimeSeconds() - timer.getElapsedTimeSeconds()) > 0.5){
                    switchState(IntakeStateMachineStates.SECOND_ARTIFACT);
                }
                break;
            case SECOND_ARTIFACT:
                intake.intakeNextArtifacts();;
                if(intake.thirdArtifactIn()
                        || Math.abs(currTime.getElapsedTimeSeconds() - timer.getElapsedTimeSeconds()) > 0.5){
                    switchState(IntakeStateMachineStates.FINAL);
                }
                break;
            case FINAL:
                intake.stopArtifacts();
                if(canShoot){switchState(IntakeStateMachineStates.SHOOTING);}
                break;
            case SHOOTING:
                intake.shootArtifacts();
                if(!canShoot
                        || Math.abs(currTime.getElapsedTimeSeconds() - timer.getElapsedTimeSeconds()) > 5){
                    switchState(IntakeStateMachineStates.INIT);
                }
                break;
        }
    }

    public void switchState(IntakeStateMachineStates state){
        this.state = state;
        timer.resetTimer();
    }
}
