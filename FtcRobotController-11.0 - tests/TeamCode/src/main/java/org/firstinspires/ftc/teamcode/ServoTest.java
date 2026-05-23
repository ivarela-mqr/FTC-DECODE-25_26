package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Debouncer;


@TeleOp(name = "Servo SPM Test", group = "Test")
public class ServoTest extends OpMode {
    public Debouncer velDebouncer = new Debouncer(300);

    private Servo testServo1;
    private Servo testServo2;
    @Override
    public void init() {
        testServo1 = hardwareMap.get(Servo.class, "testServo1");
        testServo2 = hardwareMap.get(Servo.class, "testServo2");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.cross && velDebouncer.isReady()) {
            // Only runs on the exact frame you press Cross
            if (testServo1.getPosition()>0.8 && testServo2.getPosition()>0.8){
                testServo1.setPosition(0);
                testServo2.setPosition(0);
            }
            else{testServo1.setPosition(1);
                testServo2.setPosition(1);
            }
        }


        telemetry.addData("Servo Position", "%.3f", testServo1.getPosition());
        telemetry.addData("X", gamepad1.cross ? "PRESSED" : "released");
    }
}