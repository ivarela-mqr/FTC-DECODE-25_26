package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp

public class IntakePruebaTeleOpMode extends OpMode {
    private DcMotorEx intake;
    private double maxVel = 2600;
    private boolean dir=true;



    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class,"intake");

    }

    @Override
    public void loop() {
        if(dir){
            intake.setVelocity(gamepad1.right_trigger*maxVel);
        }else{
            intake.setVelocity(-gamepad1.right_trigger*maxVel);
        }

        if (gamepad1.cross)
            dir=!dir;

    }
}
