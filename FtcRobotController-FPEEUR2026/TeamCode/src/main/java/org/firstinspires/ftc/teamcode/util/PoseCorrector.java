package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;
import org.firstinspires.ftc.teamcode.util.Constants.Alliance;

public class PoseCorrector {
    LimeLight limeLight;
    Alliance alliance;
    private Pose lastCorrection;



    public PoseCorrector(LimeLight limeLight, Alliance alliance){
        this.limeLight = limeLight;
        this.alliance = alliance;
    }

    public boolean shouldReset(Follower follower, double turretAngle){
        double margin = 5;
        Pose limelightPose = limeLight.getCorrectedVisionPos(alliance,turretAngle);
        if (limelightPose == null)
            return false;
        Pose encoderPose = follower.getPose();
        return limelightPose.distanceFrom(encoderPose) > margin;
    }

    public Pose correctPose(Follower follower, double turretAngle, Pose goalPose) {
        Pose limelightPose = limeLight.getCorrectedVisionPos(alliance, turretAngle);
        Pose encoderPose = follower.getPose();

        if (limelightPose == null || encoderPose == null) return null;

        double distanceToGoal = encoderPose.distanceFrom(goalPose);
        double r = distanceToGoal < 40  ? 0.2  :
                distanceToGoal < 80  ? 0.1  :
                        distanceToGoal < 120 ? 0.05 : 0;

        lastCorrection = new Pose(
                encoderPose.getX() + r * (limelightPose.getX() - encoderPose.getX()),
                encoderPose.getY() + r * (limelightPose.getY() - encoderPose.getY()),
                follower.getHeading()
        );
        return lastCorrection;
    }
}
