package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

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
    public void correctPose(Follower follower, double turretAngle, Pose goalPose){
        Pose limelightPose = limeLight.getCorrectedVisionPos(alliance,turretAngle);
        Pose encoderPose = follower.getPose();

        double r; //ratio correction
        double distanceToGoal = encoderPose.distanceFrom(goalPose);
        if(distanceToGoal < 40)
            r = 0.2;
        else if(distanceToGoal < 80)
            r = 0.1;
        else if (distanceToGoal < 120)
            r = 0.05;
        else
            r = 0;

        follower.setPose( new Pose(
                encoderPose.getX() + r * (limelightPose.getX() - encoderPose.getX()),
                encoderPose.getY() + r * (limelightPose.getY() - encoderPose.getY()),
                follower.getHeading()
        ));

        lastCorrection = follower.getPose();
    }
}
