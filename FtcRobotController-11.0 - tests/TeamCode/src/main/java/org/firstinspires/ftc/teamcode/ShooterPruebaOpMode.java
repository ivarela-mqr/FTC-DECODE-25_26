package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ShooterPruebaOpMode extends OpMode {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2, intake;
    private final double maxVel = 2600;
    private double velocity = 1600;
    private int shooting = 0;

    private long lastChangeTime = 0; // time of last velocity change (in ms)
    private long cooldown = 500; // 0.5 seconds
    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter2");
        intake=hardwareMap.get(DcMotorEx.class,"intake");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        long currentTime = System.currentTimeMillis();

        shooter1.setVelocity(shooting*velocity);
        shooter2.setVelocity(shooting*velocity);

        if(gamepad1.cross && currentTime - lastChangeTime > 2 * cooldown){
            shooting = shooting == 0 ? 1 : 0;
            lastChangeTime = currentTime;
        }

        if(gamepad1.dpad_up && currentTime - lastChangeTime > cooldown) {
            velocity = maxVel - velocity > 0 ? velocity + 50 : maxVel;
            lastChangeTime = currentTime;
        }

        if(gamepad1.dpad_down && currentTime - lastChangeTime > cooldown){
            velocity = velocity > 0 ? velocity - 50 : 0;
            lastChangeTime = currentTime;
        }

        if (gamepad1.circle && currentTime - lastChangeTime > cooldown){
            if(intake.getPower()>0.1){
                intake.setPower(0);
            }else{
                intake.setPower(-1);
            }
            lastChangeTime = currentTime;
        }

        telemetry.addData("Velocity1", shooter1.getVelocity());
        telemetry.addData("Velocity2", shooter2.getVelocity());
        telemetry.update();
    }

}
