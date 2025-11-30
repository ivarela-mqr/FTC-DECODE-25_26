package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous
public class RotatingVisionOpMode extends OpMode {
     private AprilTagWebcam aprilTagWebcam;
     private DcMotorEx motor;
     private double xRotCamera;



    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap,telemetry);
        motor=hardwareMap.get(DcMotorEx.class,"rotor");
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id22 =aprilTagWebcam.getTagBySpecificId(22);
        if(id22 != null) {
            aprilTagWebcam.display(id22);
            xRotCamera=id22.ftcPose.yaw;
        }
    }
}