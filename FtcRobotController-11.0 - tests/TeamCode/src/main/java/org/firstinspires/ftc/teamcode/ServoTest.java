package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Debouncer;


@TeleOp(name = "Servo SPM Test", group = "Test")
public class ServoTest extends OpMode {
    public Debouncer velDebouncer = new Debouncer(300);

    private Servo testServo;
    @Override
    public void init() {
        testServo = hardwareMap.get(Servo.class, "testServo");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.cross && velDebouncer.isReady()) {
            // Only runs on the exact frame you press Cross
            if (testServo.getPosition()>0.8){
                testServo.setPosition(0);
            }else{testServo.setPosition(1);}
        }


        telemetry.addData("Servo Position", "%.3f", testServo.getPosition());
        telemetry.addData("X", gamepad1.cross ? "PRESSED" : "released");
    }
}