package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debouncer;

@TeleOp(name="Turret PID Tuner", group="Test")
public class TurretPIDAdjust extends OpMode {

    private CRServo rotorL, rotorR;
    private DcMotorEx encoder;   // Solo para leer el encoder

    // ===== LIMITES =====
    final int LIMITE_MAX = 10000;
    final int LIMITE_MIN = -10000;

    // ===== PID =====
    double kP = 0.0006;
    double kI = 0.0;
    double kD = 0.00015;

    double integral = 0;
    double lastError = 0;

    int targetPosition = 0;
    double lastTime = 0;
    DriveTrain driveTrain;
    IMU imu;
    LimeLight limelight;
    Debouncer debouncer;
    @Override
    public void init() {

        rotorL = hardwareMap.get(CRServo.class, "rotorL");
        rotorR = hardwareMap.get(CRServo.class, "rotorR");

        encoder = hardwareMap.get(DcMotorEx.class, "intake");

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastTime = getRuntime();
        driveTrain = new DriveTrain(hardwareMap);
        limelight = new LimeLight(hardwareMap, Constants.Alliance.BLUE);
        imu = hardwareMap.get(IMU.class,"imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        debouncer = new Debouncer(500);
    }

    @Override
    public void loop() {

        // ===== dt =====
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // ===== CONTROL TARGET CON STICK =====
        double stickInput = -gamepad1.left_stick_x;
        targetPosition += stickInput * 40;   // sensibilidad

        targetPosition = Math.max(LIMITE_MIN,
                Math.min(LIMITE_MAX, targetPosition));

        // ===== AJUSTE PID EN VIVO =====
        if (gamepad1.dpad_up && debouncer.isReady()) kP += 0.005;
        if (gamepad1.dpad_down&& debouncer.isReady()) kP -= 0.005;

        if (gamepad1.dpad_right&& debouncer.isReady()) kD += 0.0005;
        if (gamepad1.dpad_left&& debouncer.isReady()) kD -= 0.0005;

        if (gamepad1.y&& debouncer.isReady()) kI += 0.00001;
        if (gamepad1.a&& debouncer.isReady()) kI -= 0.00001;

        if (gamepad1.b&& debouncer.isReady()) integral = 0;

        // ===== LECTURA ENCODER =====
        int currentPosition = encoder.getCurrentPosition();

        // ===== PID =====
        double error = targetPosition - currentPosition;

        integral += error * dt;

        // Anti-windup
        integral = Math.max(-2000, Math.min(2000, integral));

        double derivative = 0;
        if (dt > 0) {
            derivative = (error - lastError) / dt;
        }

        double output = (kP * error)
                + (kI * integral)
                + (kD * derivative);

        lastError = error;

        // Deadband
        if (Math.abs(error) < 5) {
            output = 0;
        }

        // Limitar potencia
        output = Math.max(-1, Math.min(1, output));

        // ===== APLICAR A LOS DOS SERVOS =====
        rotorL.setPower(output);
        rotorR.setPower(output);

        // ===== TELEMETRY =====
        telemetry.addData("Target", targetPosition);
        telemetry.addData("Position", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("Power", output);
        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.update();
    }
}

