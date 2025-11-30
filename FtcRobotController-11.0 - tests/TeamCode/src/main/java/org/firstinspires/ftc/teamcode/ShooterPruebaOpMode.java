package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ShooterPruebaOpMode extends OpMode {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double maxVel = 2600;

    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter2");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        shooter1.setVelocity(gamepad1.left_trigger*maxVel);
        shooter2.setVelocity(gamepad1.left_trigger*maxVel);


        telemetry.addData("Velocity",shooter2.getVelocity());
        telemetry.update();
    }

}
