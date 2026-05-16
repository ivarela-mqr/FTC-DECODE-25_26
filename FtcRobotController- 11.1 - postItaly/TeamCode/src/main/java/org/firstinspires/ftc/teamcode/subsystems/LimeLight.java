package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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
    // Añade estas constantes (las calibras tú midiendo)
    private static final double LL_ORIGIN_OFFSET_X = -0.3856325149536133; // en pulgadas
    private static final double LL_ORIGIN_OFFSET_Y = -0.03463870659470558; // en pulgadas

    public Pose getCorrectedVisionPos(Constants.Alliance alliance, double turretAngle) {
        LLResult llResult = limelight.getLatestResult();
        if (llResult == null || !llResult.isValid()) return null;

        Pose3D pose;
        pose = llResult.getBotpose();
        if (pose == null) return null;

        // Limelight da metros → convertir a pulgadas
        double x = pose.getPosition().toUnit(DistanceUnit.INCH).x;
        double y = pose.getPosition().toUnit(DistanceUnit.INCH).y;

        // WPI pone el origen en el CENTRO del campo, Pedro Pathing en la ESQUINA
        // El campo mide 144" x 144"
        x += 72.0;
        y = 72.0 - y;

        // Corrección por offset de torreta
        double radius = 6.5;
        x -= Math.cos(turretAngle) * radius;
        y += Math.sin(turretAngle) * radius;

        double heading = pose.getOrientation().getYaw(); // ya en grados

        return new Pose(x, y, Math.toRadians(heading));
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

