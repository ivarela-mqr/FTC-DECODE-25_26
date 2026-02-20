package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ShootingStateMachine {
    Shooter shooter;
    Pose shootingPose = new Pose(56, 93);
    public enum States {
        INIT,
        LOADING,
        SHOOTING,
        INTAKING
    }
    public States state;
    public IntakeStateMachine intakeStateMachine = new IntakeStateMachine();
    public boolean isInShootingPos = false;
    boolean canShoot = false;
    public Timer timer, actualTime;

    static final double g = 9.81;

    public static class ShotResult {
        public double velocity;   // m/s
        public double angleDeg;   // grados
        public double rpm;        // RPM rueda
        public double servoPos;
    }
    static double shooterHeightInches = 10;
    static double basketHeightInches = 25;
    static double wheelDiameterInches = 3.75;
    public Timer init;

    // Convierte ángulo a servo (posición 0 = 24°, posición 1 = 50°)
    public static double angleToServo(double angleDeg) {
        double minAngle = 24.0; // servo 0
        double maxAngle = 50.0; // servo 1
        double servoPos = (angleDeg - minAngle) / (maxAngle - minAngle);
        return Math.max(0.0, Math.min(1.0, servoPos));
    }

    // Calcula tiro parabólico exacto
    public static ShotResult calculateShot(
            Pose shooterPose, Pose basketPose
    ) {
        ShotResult result = new ShotResult();

        // Convertir pulgadas a metros
        double shooterHeight = shooterHeightInches * 0.0254;
        double basketHeight = basketHeightInches * 0.0254;
        double wheelRadius = (wheelDiameterInches / 2.0) * 0.0254;

        // Distancia horizontal entre robot y canasta (metros)
        double dx = Math.hypot(
                (basketPose.getX() - shooterPose.getX()) * 0.0254,
                (basketPose.getY() - shooterPose.getY()) * 0.0254
        );

        // Diferencia de altura
        double dy = basketHeight - shooterHeight;

        // Ángulo intermedio: 40° (punto medio entre bombeado y plano)
        double angleDeg = 45;
        double theta = Math.toRadians(angleDeg);

        double cosTheta = Math.cos(theta);

        // Velocidad inicial requerida
        double v = Math.sqrt(g * dx * dx / (2 * cosTheta * cosTheta * (dx * Math.tan(theta) - dy)));

        // RPM de la rueda
        double rpm = v / wheelRadius * 60.0 / (2.0 * Math.PI);

        // Posición del servo
        double servoPos = angleToServo(angleDeg);

        result.velocity = v;
        result.rpm = rpm;
        result.servoPos = servoPos;
        result.angleDeg = angleDeg;

        return result;
    }
    public void init(HardwareMap hardwareMap, Constants.Alliance alliance, Pose shootingPose, Pose targetPose, Telemetry telemetry){
        //ShotResult result = calculateShot(shootingPose,targetPose);
        shooter = new Shooter(hardwareMap, alliance,0,0); //result.rpm, result.servoPos);
        timer = new Timer();
        actualTime = new Timer();
        switchState(States.INIT);
        intakeStateMachine.init(hardwareMap);
        //telemetry.addData("Target vel", result.velocity);
        //telemetry.addData("Target rpm", result.rpm);
        //telemetry.addData("Target angle", result.angleDeg);
        //telemetry.addData("Target pos servo", result.servoPos);
    }

    public void update(Pose pose, Telemetry telemetry, double yawAngle, boolean isBussyIntake){
        isInShootingPos = canShoot(pose);
        shooter.aimWithLimelight(yawAngle);
        intakeStateMachine.updateIntakeStateMachine(canShoot);
        actualTime.resetTimer();
        switch (state){
            case INIT:
                shooter.closeBlock();
                shooter.calm();
                if(isInShootingPos) {
                    shooter.preload(telemetry, yawAngle);
                    state = States.LOADING;
                }
                break;
            case LOADING:
                if(shooter.isReady()) {
                    switchState(States.SHOOTING);
                }
                else{
                    shooter.preload(telemetry,yawAngle);
                }
                break;
            case SHOOTING:
                shooter.openBlock();
                if(shooter.getBlockPos() <= 0.5)
                    canShoot = true;
                if(!intakeStateMachine.isShooting()){    //intake.numArtifactsIn(telemetry) == 0
                    canShoot = false;
                    switchState(States.INTAKING);
                }
                break;
            case INTAKING:
                shooter.closeBlock();
                shooter.calm();
                if(!intakeStateMachine.isBusy())
                    switchState(States.INIT);
                break;
            default:
                break;
        }
    }
    public boolean canShoot(Pose pose) {
        double posTolerance = 40;            // tolerancia en pulgadas (más estricta)

        double dx = pose.getX() - shootingPose.getX();
        double dy = pose.getY() - shootingPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        return distance <= posTolerance ;
    }
    public boolean canShoot(){
        return canShoot;
    }
    public boolean isBusy(){
        return (state != States.INTAKING);
    }
    public void switchState(States state){
        this.state = state;
        timer.resetTimer();
    }
}
