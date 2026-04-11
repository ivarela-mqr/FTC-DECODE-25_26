package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.Constants;

@TeleOp
public class CalculateVel extends OpMode {
    Shooter shooter;
    Intake intake;
    double currVel = 1500;
    double pos = 0;
    IMU imu;
    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        shooter = new Shooter(hardwareMap, Constants.Alliance.BLUE,currVel);
        intake = new Intake(hardwareMap);
        shooter.openBlock();
    }

    @Override
    public void loop() {
        shooter.adjustCover(pos);

        if(gamepad1.dpadUpWasPressed())
            currVel += 50;
        if(gamepad1.dpadDownWasPressed())
            currVel -= 50;
        if(gamepad1.dpadLeftWasPressed())
            pos -= 0.05;
        if(gamepad1.dpadRightWasPressed())
            pos += 0.05;

        intake.shootArtifacts();

        if(currVel - shooter.shooter0.getVelocity() > 100){
            shooter.shooter1.setPower(1);
            shooter.shooter0.setPower(1);
        }else{
            shooter.shooter1.setVelocity(currVel);
            shooter.shooter0.setVelocity(currVel);
        }
        double orientation = imu.getRobotYawPitchRollAngles().getYaw();
        telemetry.addData("Target Vel",currVel);
        telemetry.addData("Actual Vel",shooter.shooter0.getVelocity());
        telemetry.addData("Pos",pos);
        telemetry.addData("Distance",shooter.limeLight.getGoalAprilTagData(orientation)[0]);
    }
}
