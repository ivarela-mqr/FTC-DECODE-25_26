package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class ShooterPruebaOpMode extends OpMode {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private double maxVel = 2600;

    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter2");
    }

    @Override
    public void loop() {
        shooter1.setVelocity(gamepad1.left_trigger*maxVel);
        shooter1.setVelocity(gamepad1.left_trigger*maxVel);
        shooter2.setVelocity(-gamepad1.left_trigger*maxVel);
    }

}
