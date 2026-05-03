package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Debouncer;


@TeleOp(name = "Servo SPM Test", group = "Test")
public class TurretTest extends OpMode {
    public Debouncer velDebouncer = new Debouncer(300);

    private Servo rotorR, rotorL;

    @Override
    public void init() {
        rotorR = hardwareMap.get(Servo.class, "testServo");
        rotorL = hardwareMap.get(Servo.class, "testServo");
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.cross && velDebouncer.isReady()) {
            // Only runs on the exact frame you press Cross
            if (rotorR.getPosition()>0.8 ){
                rotorR.setPosition(0);
                rotorL.setPosition(0);
            }else{
                rotorR.setPosition(1);
                rotorL.setPosition(1);
            }
        }


        telemetry.addData("Servo Position", "%.3f", rotorR.getPosition());
        telemetry.addData("X", gamepad1.cross ? "PRESSED" : "released");
    }

}