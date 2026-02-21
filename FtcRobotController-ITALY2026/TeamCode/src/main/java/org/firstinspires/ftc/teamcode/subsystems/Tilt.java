package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Tilt {
    CRServo climb1, climb2, climb3, climb4;
    Timer timer = new Timer();
    Timer actualTimer = new Timer();
    public Tilt(HardwareMap hardwareMap){
        climb1 = hardwareMap.get(CRServo.class, "climb1");
        climb2 = hardwareMap.get(CRServo.class, "climb2");
        climb3 = hardwareMap.get(CRServo.class, "climb3");
        climb4 = hardwareMap.get(CRServo.class, "climb4");
    }

    public void Teleop(Gamepad gamepad){
        actualTimer.resetTimer();
        if(gamepad.dpadUpWasPressed()){
            timer.resetTimer();
            climb1.setPower(-1);
            climb2.setPower(-1);
            climb3.setPower(1);
            climb4.setPower(1);
        }
        if(Math.abs(timer.getElapsedTimeSeconds() - actualTimer.getElapsedTimeSeconds()) > 2.5){
            climb1.setPower(0);
            climb2.setPower(0);
            climb3.setPower(0);
            climb4.setPower(0);
        }
    }
}
