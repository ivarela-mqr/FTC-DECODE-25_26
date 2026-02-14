package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@TeleOp
public class PruebaPIDShooter extends OpMode {
    DcMotorEx shooter1, shooter2;
    double highVel = 1800;
    double lowVel = 1000;
    double currVel = highVel;
    double F = 0, P = 0;
    double[] steps = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 0;
    @Override
    public void init() {
        shooter1=hardwareMap.get(DcMotorEx.class,"shooter1");
        shooter2=hardwareMap.get(DcMotorEx.class,"shooter0");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients coeficients = new PIDFCoefficients(P,0,0,F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,coeficients);
    }

    @Override
    public void loop() {

        if(gamepad1.yWasPressed()){
            if(currVel == highVel)
                currVel = lowVel;
            else
                currVel = highVel;
        }

        if(gamepad1.bWasPressed())
            stepIndex = (stepIndex + 1) % steps.length;

        if(gamepad1.dpadLeftWasPressed())
            F += steps[stepIndex];
        if(gamepad1.dpadRightWasPressed())
            F -= steps[stepIndex];
        if(gamepad1.dpadUpWasPressed())
            P += steps[stepIndex];
        if(gamepad1.dpadDownWasPressed())
            P -= steps[stepIndex];
        PIDFCoefficients coeficients = new PIDFCoefficients(P,0,0,F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER,coeficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_WITHOUT_ENCODER,coeficients);
        shooter1.setVelocity(currVel);
        shooter2.setVelocity(currVel);
        double error = currVel - shooter1.getVelocity();
        telemetry.addData("Target vel", currVel);
        telemetry.addData("Actual vel", shooter1.getVelocity());
        telemetry.addData("Error","%.2f" ,error);
        telemetry.addData("Tuning P","%.4f (D-Pad U/D)" ,P);
        telemetry.addData("Tuning F","%.4f (D-Pad U/D)" ,F);
        telemetry.addData("Step","%.4f (B Button)" ,steps[stepIndex]);
    }
}
