package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LimeLight {
    private final Limelight3A limelight;
    public LimeLight(HardwareMap hardwareMap, Constants.Alliance alliance) {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        if (alliance == Constants.Alliance.BLUE)
            limelight.pipelineSwitch(0); // Blue alliance aprilTag
        else if (alliance == Constants.Alliance.RED)
            limelight.pipelineSwitch(9); // Red alliance aprilTag

        limelight.start();
    }

    public void switchAlliance(Constants.Alliance newAlliance){
        if (newAlliance == Constants.Alliance.BLUE)
            limelight.pipelineSwitch(8); // Blue alliance aprilTag
        else if (newAlliance == Constants.Alliance.RED)
            limelight.pipelineSwitch(9); // Red alliance aprilTag
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
        data[0] = 10000;
        data[1] = 0;
        return data;
    }
    public Pose resetPose(double yawAngle){
        limelight.updateRobotOrientation(yawAngle);
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D pose = llResult.getBotpose();
            return new Pose(pose.getPosition().x,pose.getPosition().y,pose.getOrientation().getYaw());
        }
        return null;
    }

    public Pose getCorrectedVisionPos(Constants.Alliance alliance, double turretAngle){ //Robot pose according to camera (corrected)
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid())
            return null;

        Pose3D pose = llResult.getBotpose();
        Pose corrPose;

        double x = pose.getPosition().x;
        double y = pose.getPosition().y;
        double dirX = Math.cos(turretAngle);
        double dirY = Math.sin(turretAngle);


        double radius = 6.5;

        double dx = dirX * radius;
        double dy = dirY * radius;
        corrPose = new Pose(
                x-dx,
                y-dy,
                pose.getOrientation().getYaw()
        );
        return corrPose;
    }
    public Pose getRawVisionPose(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid())
            return null;

        Pose3D pose = llResult.getBotpose();
        return new Pose(pose.getPosition().x, pose.getPosition().y, 0);
    }
    private double getDistanceFromTargeta(double ta){
        return 180.5062* Math.pow(ta,-0.5018798);
    }
}

