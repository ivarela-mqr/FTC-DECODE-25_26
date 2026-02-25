package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.IntakeStateMachineStates;
import org.opencv.core.Mat;

public class IntakeStateMachine {
    public Intake intake;
    public IntakeStateMachineStates state;
    public Timer timer = new Timer();
    public Timer currTime = new Timer();

    public void init(HardwareMap hardwareMap){
        intake = new Intake(hardwareMap);
        state = IntakeStateMachineStates.INIT;
    }
    public boolean isBusy() {
        return state != IntakeStateMachineStates.INIT && state != IntakeStateMachineStates.FIRST_ARTIFACT && state != IntakeStateMachineStates.SECOND_ARTIFACT;
    }
    public boolean isIntaking() {
        return state == IntakeStateMachineStates.INIT
                || state == IntakeStateMachineStates.FIRST_ARTIFACT
                || state == IntakeStateMachineStates.SECOND_ARTIFACT;
    }

    public boolean isFull(){
        return state == IntakeStateMachineStates.FINAL || state == IntakeStateMachineStates.SHOOTING;
    }
    public boolean isShooting() {
        return intake.numArtifactsIn() > 0;
    }

    public void updateIntakeStateMachine(boolean canShoot, Gamepad gamepad1, Gamepad gamepad2){
        currTime.resetTimer();
        intake.TeleOp(gamepad1,gamepad2,isIntaking());

        if(gamepad2.dpadDownWasPressed())
            switchState(IntakeStateMachineStates.FINAL);

        if(gamepad2.triangle)
            switchState(IntakeStateMachineStates.UNLOAD);

        switch (state){
            case INIT:
                intake.intakeFirstArtifact();
                if(intake.firstArtifactIn()){
                    switchState(IntakeStateMachineStates.FIRST_ARTIFACT);
                }
                break;
            case FIRST_ARTIFACT:
                intake.intakeNextArtifacts();
                if(intake.secondArtifactIn()){
                    intake.transferPosition2ArtifactIn = intake.transfer.getCurrentPosition();
                    switchState(IntakeStateMachineStates.SECOND_ARTIFACT);
                    intake.setTransferPosition(50);
                }
                break;
            case SECOND_ARTIFACT:
                intake.intakeNextArtifacts();
                if(intake.thirdArtifactIn()
                        && Math.abs(timer.getElapsedTimeSeconds() - currTime.getElapsedTimeSeconds()) > 1){
                    switchState(IntakeStateMachineStates.FINAL);
                }
                break;
            case FINAL:
                intake.stopArtifacts();
                if(canShoot){
                    switchState(IntakeStateMachineStates.SHOOTING);
                }
                break;
            case SHOOTING:
                intake.shootArtifacts();
                if(Math.abs(timer.getElapsedTimeSeconds() - currTime.getElapsedTimeSeconds())> 1){
                    switchState(IntakeStateMachineStates.INIT);
                }
                break;
            case UNLOAD:
                if(Math.abs(timer.getElapsedTimeSeconds() - currTime.getElapsedTimeSeconds()) > 0.5)
                    switchState(IntakeStateMachineStates.INIT);
                intake.unload();
                break;
        }
    }

    public void switchState(IntakeStateMachineStates state){
        this.state = state;
        timer.resetTimer();
    }
}
