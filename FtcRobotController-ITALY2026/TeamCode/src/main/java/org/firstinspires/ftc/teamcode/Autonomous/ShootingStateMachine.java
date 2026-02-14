package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ShootingStateMachine {
    Shooter shooter;
    Intake intake;
    Pose shootingPose = new Pose(72.510, 84.611);
    public enum States {
        INIT,
        LOADING,
        SHOOTING,
        INTAKING
    }
    public States state;
    public boolean isInShootingPos = false;

    public void init(HardwareMap hardwareMap, Constants.Alliance alliance){
        shooter = new Shooter(hardwareMap, alliance);
        intake = new Intake(hardwareMap);
        state = States.INIT;
    }

    public void update(Pose pose, Telemetry telemetry, double yawAngle){
        isInShootingPos = canShoot(pose);
        switch (state){
            case INIT:
                shooter.closeBlock();
                //intake.intake();
                shooter.calm();
                if(isInShootingPos) {
                    shooter.preload(telemetry, yawAngle);
                    state = States.LOADING;
                }
                break;
            case LOADING:
                if(shooter.isReady()) {
                    state = States.SHOOTING;
                }
                else{
                    shooter.preload(telemetry,yawAngle);
                    shooter.openBlock();
                }
                break;
            case SHOOTING:
                if(intake.firstArtifactIn() && shooter.getBlockPos() >= 0.5){
                    intake.shootArtifacts();
                }else if(!intake.firstArtifactIn()){
                    state = States.INTAKING;
                }
                break;
            case INTAKING:
                shooter.closeBlock();
                if(intake.firstArtifactIn()){
                    state = States.INIT;
                }else {
                    intake.intake();
                }
                break;
            default:
                break;
        }
    }
    public boolean canShoot(Pose pose) {
        double posTolerance = 10;            // tolerancia en pulgadas (más estricta)

        double dx = pose.getX() - shootingPose.getX();
        double dy = pose.getY() - shootingPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);


        return distance <= posTolerance ;
    }

    public boolean isBusy(){
        return state != States.INTAKING;
    }
}
