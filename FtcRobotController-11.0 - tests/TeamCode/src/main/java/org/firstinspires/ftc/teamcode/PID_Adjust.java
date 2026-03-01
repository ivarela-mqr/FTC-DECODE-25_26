package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.apache.commons.math3.analysis.function.Max;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Turret PID Tuner", group="Test")
@Config
public class PID_Adjust extends OpMode {

    private CRServo rotorL, rotorR;
    private DcMotorEx encoder;   // Solo para leer el encoder

    // ===== LIMITES =====
    final int LIMITE_MAX = 10000;
    final int LIMITE_MIN = -10000;

    // ===== PID =====
    public static double kP = 0;
    public static double kI = 0.0;
    public static double kD = 0;
    double lastError = 0;
    int targetPosition = 0;
    double lastTime = 0;
    double offset = 0;
    IMU imu;
    Limelight3A limelight;
    double error_filtrado = 0;
    public static double alfa = 0.9;
    boolean hold = true;
    @Override
    public void init() {

        rotorL = hardwareMap.get(CRServo.class, "rotorL");
        rotorR = hardwareMap.get(CRServo.class, "rotorR");

        encoder = hardwareMap.get(DcMotorEx.class, "intake");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = getRuntime();
        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(8); // Blue alliance aprilTag
        limelight.start();
        imu = hardwareMap.get(IMU.class,"imu");
    }

    @Override
    public void loop() {

        // ===== dt =====
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;
        offset = getGoalAprilTagData(telemetry,imu.getRobotYawPitchRollAngles().getYaw())[0];
        error_filtrado = alfa * error_filtrado + (1 - alfa) * offset;

        // ===== LECTURA ENCODER =====
        int currentPosition = encoder.getCurrentPosition();


        // Anti-windup

        double derivative = 0;
        if (dt > 0) {
            derivative = (error_filtrado - lastError) / dt;
        }

        double output = (kP * error_filtrado)
                + (kD * derivative);

        lastError = error_filtrado;

        // Deadband
        if (!hold && Math.abs(error_filtrado) < 3){
            output = 0;
            hold = true;
        }else if(hold && Math.abs(error_filtrado) > 5){
            hold = false;
        }

        // Limitar potencia
        output = Math.max(-1, Math.min(1, output));

        // ===== APLICAR A LOS DOS SERVOS =====
        double posR = encoder.getCurrentPosition();
        if ((posR <= LIMITE_MIN && output > 0) ||
            (posR >= LIMITE_MAX && output < 0) || hold) {
            rotorL.setPower(0);
            rotorR.setPower(0);
        }else {
            rotorL.setPower(-output);
            rotorR.setPower(-output);
        }
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("Error", offset);
        dashboardTelemetry.addData("Limit",0);
        // ===== TELEMETRY =====
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Position", currentPosition);
        telemetry.addData("Error", offset);
        telemetry.addData("Error filtrado", error_filtrado);
        telemetry.addData("Power", output);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }

    public double[] getGoalAprilTagData(Telemetry telemetry, double yawAngle){
        limelight.updateRobotOrientation(yawAngle);
        LLResult llResult = limelight.getLatestResult();
        telemetry.addData("REsult",llResult.getTx());
        double[] data = new double[2];
        if (llResult != null && llResult.isValid()){
            telemetry.addData("PATATA",llResult.getTx());
            //Pose3D botPose = llResult.getBotpose();
            //telemetry.addData("Tx", llResult.getTx());
            //telemetry.addData("Ty", llResult.getTy());
            //telemetry.addData("Ta", llResult.getTa());
            //telemetry.addData("distance in cm",getDistanceFromTargeta(llResult.getTa()));

            data[0] = llResult.getTx();
            data[1] = 0;
            return  data;
        }
        data[0] = 0;
        data[1] = 0;
        return data;
    }
}

