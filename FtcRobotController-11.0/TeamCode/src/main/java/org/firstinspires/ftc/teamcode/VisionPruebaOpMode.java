package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class VisionPruebaOpMode extends OpMode {
    AprilTagWebcam aprilTagWebcam =new AprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap,telemetry);

    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id22 =aprilTagWebcam.getTagBySpecificId(22);
        if(id22 != null)
//            telemetry.addData("id20 String", id22.toString());
            aprilTagWebcam.display(id22);

    }

}
