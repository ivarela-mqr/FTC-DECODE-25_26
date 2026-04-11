package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimeLight {
    private final Limelight3A limelight;
    public LimeLight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(8);
        limelight.start();
    }
    public double[] getGoalAprilTagData(double yawAngle){
        limelight.updateRobotOrientation(yawAngle);
        LLResult llResult = limelight.getLatestResult();
        double[] data = new double[2];
        if (llResult != null && llResult.isValid()){
            //Pose3D botPose = llResult.getBotpose();
            //telemetry.addData("Tx", llResult.getTx());
            //telemetry.addData("Ty", llResult.getTy());
            //telemetry.addData("Ta", llResult.getTa());
            //telemetry.addData("distance in cm",getDistanceFromTargeta(llResult.getTa()));

            data[0] = llResult.getTx();
            data[1] = getDistanceFromTargeta(llResult.getTa());
            return  data;
        }
        data[0] = 0;
        data[1] = 0;
        return data;
    }
    private double getDistanceFromTargeta(double ta){
        return 180.5062* Math.pow(ta,-0.5018798);
    }
}

