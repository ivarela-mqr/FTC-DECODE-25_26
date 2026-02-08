package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@TeleOp
public class Italy26TeleOpMode extends OpMode {
    DriveTrain driveTrain;
    Intake intake;
    Shooter shooter;
    IMU imu;
    YawPitchRollAngles orientation;
    double yawOffset = 0;

    @Override
    public void init() {
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, Constants.Alliance.BLUE, telemetry);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
    }

    @Override
    public void start(){
        gamepad1.rumble(20);
        gamepad2.rumble(20);
    }

    @Override
    public void loop() {
        orientation = imu.getRobotYawPitchRollAngles();
        double yawAngle = orientation.getYaw(AngleUnit.DEGREES) - yawOffset;
        if(gamepad1.options){
            yawOffset = orientation.getYaw();
        }

        driveTrain.TeleOp(gamepad1,telemetry,yawAngle);
        intake.TeleOp(gamepad1, telemetry, yawAngle);
        shooter.TeleOp(gamepad1, telemetry, yawAngle, intake.getIsShooting());
        telemetry.addData("Is shooting", intake.getIsShooting());
        telemetry.update();


    }
}
