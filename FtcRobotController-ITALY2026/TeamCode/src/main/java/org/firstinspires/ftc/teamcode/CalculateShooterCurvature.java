package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.security.Provider;
@TeleOp
public class CalculateShooterCurvature extends OpMode {
    private LimeLight limelight;
    private IMU imu;
    Servo coverL, coverR, block;
    private CRServo rotorL,rotorR; // Servo 1 con encoder
    double distance = 0;
    double F = 1;
    double p = 0;
    double[] steps = {0.1, 0.01, 0.001};
    int index =0;
    int stepsVel = 50;
    DcMotorEx shooter1, shooter2;
    DcMotorEx  intake, transfer;
    double highVel = 1300;
    double currVel = highVel;
    ColorSensor colorSensor;
    ColorSensor.DetectedColors detectedColor;
    DriveTrain driveTrain;

    DcMotorEx encoder;

    // PID state rotors
    double integralSum_rotor = 0;
    double lastError_rotor = 0;
    long lastTime_rotor = 0;

    // PID constants rotors
    double kP_rotor = 0.02;
    double kI_rotor = 0.0;
    double kD_rotor = 0.002;
    // Servo state rotors
    double rotorPos = 0.5;
    DistanceSensor distanceSensor1, distanceSensor2;
    final int LIMITE_IZQUIERDA = -10000;
    final int LIMITE_DERECHA = 10000;

    // Factor de movimiento
    final double VELOCIDAD_FACTOR = - 0.05;
    double offset = 0;
    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake = hardwareMap.get(DcMotorEx.class,"intake");
        transfer = hardwareMap.get(DcMotorEx.class,"transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients coeficients = new PIDFCoefficients(1,0,0,15.3);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
        limelight = new LimeLight(hardwareMap, Constants.Alliance.BLUE);
        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        coverL = hardwareMap.get(Servo.class,"coverL");
        coverR = hardwareMap.get(Servo.class,"coverR");


        rotorL = hardwareMap.get(CRServo.class,"rotorL");
        rotorR = hardwareMap.get(CRServo.class,"rotorR");

        block = hardwareMap.get(Servo.class,"block");
        coverL.setDirection(Servo.Direction.REVERSE);
        colorSensor = new ColorSensor();
        colorSensor.init(hardwareMap);
        driveTrain = new DriveTrain(hardwareMap);

        distanceSensor1 = hardwareMap.get(DistanceSensor.class,"distanceSensor1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class,"distanceSensor2");
        encoder = hardwareMap.get(DcMotorEx.class, "intake");

// Reseteamos el encoder al inicio
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// Cambiamos a modo de correr usando encoder
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        offset = limelight.getGoalAprilTagData(orientation.getYaw())[0];
        driveTrain.TeleOp(gamepad1,telemetry, orientation.getYaw());
        if(gamepad1.dpadLeftWasPressed())
            F += steps[index];
        if(gamepad1.dpadRightWasPressed())
            F -= steps[index];

        if(gamepad1.dpadUpWasPressed())
            p += steps[index];
        if(gamepad1.dpadDownWasPressed())
            p -= steps[index];

        if(gamepad1.bWasPressed())
            index = (index + 1) %steps.length;

        if(gamepad1.xWasPressed())
            block.setPosition(0);

        detectedColor = colorSensor.getDetectedColor();
        //telemetry.addData("ColorDetected",detectedColor);
        //telemetry.addData("Sensor 1", distanceSensor1.getDistance(DistanceUnit.CM));
        //telemetry.addData("Sensor 2", distanceSensor2.getDistance(DistanceUnit.CM));
        /*if(detectedColor != ColorSensor.DetectedColors.UNKNOWN)
            transfer.setPower(0);*/
        //F = getPosCover(distance);
        coverL.setPosition(0.2);
        coverR.setPosition(0.2);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p,0,0,F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        if(gamepad1.right_trigger > 0.1){
            shooter1.setVelocity(currVel);
            shooter2.setVelocity(currVel);
        }
        else{
            shooter1.setPower(0);
            shooter2.setPower(0);
        }
        if(gamepad1.leftBumperWasPressed()){
            intake.setPower(1);
            transfer.setPower(1);
        }
        /*if(gamepad1.leftBumperWasPressed() && detectedColor != ColorSensor.DetectedColors.UNKNOWN)
            transfer.setPower(1);*/

        if (gamepad1.rightBumperWasPressed()){
            intake.setPower(0);
            transfer.setPower(0);
        }
        double vel = getVelocity(distance);


        // --- Control simple de servos ---
        // Usando gamepad para mover servos hacia adelante/atrás
        double power1 = gamepad2.left_stick_y;   // joystick vertical izquierdo
        double power2 = gamepad2.right_stick_y;  // joystick vertical derecho

        //-10000 Limite derecho

        aimShooterWithLimeLight(telemetry,offset);
        telemetry.addData("up",p);
        telemetry.addData("down",F);
        // Escalar potencia para servos si es necesario

        int position = encoder.getCurrentPosition();
        telemetry.addData("OFFSET", steps[index]);
        telemetry.addData("Encoder Position", position);

        telemetry.addData("Position",coverR.getPosition());
        telemetry.addData("Distance",distance);
        telemetry.addData("Velocity", vel);
        telemetry.addData("Pos", block.getPosition());
        telemetry.addData("RealVelocity",shooter1.getVelocity());
        telemetry.addData("RealVelocity",shooter2.getVelocity());
        telemetry.update();
    }
    private double getPosCover(double distance){
        return Math.min(0,Math.max(-0.0027027 * distance + 1.082164,1));
    }
    private double getVelocity(double x){
        return (int)(157.4115 +
                16.39278 * x -
                0.08673005 * Math.pow(x,2) +
                0.0001515039 * Math.pow(x,3));
    }
    private double getDistanceFromTargeta(double ta){
        return 180.5062* Math.pow(ta,-0.5018798);
    }


    public void aimShooterWithLimeLight(Telemetry telemetry, double angle){

        // If AprilTag not detected don't move
        if (Math.abs(angle) < 0.05) return;

        moverServos(angle);
    }
    public void moverServos(double offsetX) {
        // Lectura de encoders
        int posR = encoder.getCurrentPosition();

        // Calculamos potencia a partir del offset
        double potencia = offsetX * VELOCIDAD_FACTOR;

        // Verificamos límites: si cualquiera está en el límite, paramos ambos
        if ((posR <= LIMITE_IZQUIERDA && potencia > 0) ||
                (posR >= LIMITE_DERECHA && potencia < 0)) {
            potencia = 0;
        }

        // Aplicamos misma potencia a ambos servos
        rotorL.setPower(potencia);
        rotorR.setPower(potencia);

        // Telemetría para debug
        telemetry.addData("Pos R", posR);
        telemetry.addData("Potencia", potencia);
    }

    private double pidCalculate(double error) {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime_rotor) / 1e9;

        if (lastTime_rotor == 0) {
            lastTime_rotor = currentTime;
            return 0;
        }

        integralSum_rotor += error * dt;
        double derivative = (error - lastError_rotor) / dt;

        lastError_rotor = error;
        lastTime_rotor = currentTime;

        return (kP_rotor * error) + (kI_rotor * integralSum_rotor) + (kD_rotor * derivative);
    }
}
