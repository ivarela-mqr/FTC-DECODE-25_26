package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.IntakeStateMachineStates;

public class IntakeStateMachine {
    public Intake intake;
    public IntakeStateMachineStates state;
    public IntakeStateMachineStates previousState;
    public Timer timer = new Timer();
    public Timer currTime = new Timer();

    public IntakeStateMachine(HardwareMap hardwareMap){
        intake = new Intake(hardwareMap);
        state = IntakeStateMachineStates.INIT;
        previousState = IntakeStateMachineStates.END;
    }
    public boolean isFull(){
        return state == IntakeStateMachineStates.FINAL
                || state == IntakeStateMachineStates.SHOOTING
                || state == IntakeStateMachineStates.INIT;
    }
    public void TeleOp(boolean canShoot, Gamepad gamepad1, Gamepad gamepad2){
        currTime.resetTimer();

        if(gamepad2.circle)
            switchState(IntakeStateMachineStates.FINAL);

        if(gamepad2.triangle)
            switchState(IntakeStateMachineStates.UNLOAD);

        if(gamepad2.square)
            switchState((IntakeStateMachineStates.INIT));

        if(gamepad1.right_trigger > 0.1 && state != IntakeStateMachineStates.FINAL && state != IntakeStateMachineStates.SHOOTING)
            switchState(IntakeStateMachineStates.FINAL);

        switch (state){

            case INIT:
                intake.intakeFirstArtifact();
                if(intake.firstArtifactIn()){
                    switchState(IntakeStateMachineStates.FIRST_ARTIFACT);
                }
                break;
            case FIRST_ARTIFACT:
                intake.intakeNextArtifacts();

                if(intake.secondArtifactIn()
                        && Math.abs(timer.getElapsedTimeSeconds() - currTime.getElapsedTimeSeconds()) > 1){
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
                    gamepad1.rumble(1000);
                    gamepad2.rumble(1000);
                    intake.led.setPosition(0.5);
                }
                break;
            case FINAL:
                intake.stopArtifacts();
                if(canShoot){
                    intake.led.setPosition(0);
                    switchState(IntakeStateMachineStates.SHOOTING);
                    intake.led.setPosition(1);
                }
                break;
            case SHOOTING:
                intake.shootArtifacts();
                if(gamepad1.right_trigger < 0.9){
                    switchState(IntakeStateMachineStates.INIT);
                    intake.led.setPosition(0);
                }
                break;
            case UNLOAD:
                if(Math.abs(timer.getElapsedTimeSeconds() - currTime.getElapsedTimeSeconds()) > 0.5) {
                    switchState(IntakeStateMachineStates.INIT);
                }
                intake.unload();
                break;
            case END:
                intake.stopArtifacts();
            default:
                break;
        }
    }
    public void switchState(IntakeStateMachineStates state){
        previousState = state;
        this.state = state;
        timer.resetTimer();
    }
}


