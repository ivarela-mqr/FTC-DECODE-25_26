package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LimeLight {

    private Limelight3A limelight;


    LimeLight(HardwareMap hardwareMap, Constants.Alliance alliance) {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        if (alliance == Constants.Alliance.BLUE)
            limelight.pipelineSwitch(8); // Blue alliance aprilTag
        else if (alliance == Constants.Alliance.RED)
            limelight.pipelineSwitch(8); // Red alliance aprilTag

        limelight.start();
    }


    public double[] getGoalAprilTagData(Telemetry telemetry, double yawAngle){
        limelight.updateRobotOrientation(yawAngle);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            telemetry.addData("distance in cm",getDistanceFromTargeta(llResult.getTa()));
            double[] data = new double[2];
            data[0] = llResult.getTx();
            data[1] = getDistanceFromTargeta(llResult.getTa());
            return  data;
        }
        return new double[2];
    }

    private double getDistanceFromTargeta(double ta){
        return 180.5062* Math.pow(ta,-0.5018798);
    }
}

