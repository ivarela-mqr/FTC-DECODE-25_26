package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ServoTest extends OpMode {
    Servo servoL,servoR;

    @Override
    public void init() {
        servoL = hardwareMap.get(Servo.class,"servoL");
        servoR = hardwareMap.get(Servo.class,"servoR");
        servoL.setDirection(Servo.Direction.REVERSE);
        //servoL.scaleRange(0.25,0.75);
        //servoR.scaleRange(0.25,0.75);
    }

    @Override
    public void loop() {
        if(gamepad1.left_trigger > 0.9){
            servoL.setPosition(1);
            servoR.setPosition(1);
        }else{
            servoR.setPosition(0);
            servoL.setPosition(0);
        }
    }
}
