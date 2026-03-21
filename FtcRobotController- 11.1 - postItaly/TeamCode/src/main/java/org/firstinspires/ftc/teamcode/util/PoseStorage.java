package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose currentPose = null;
    public static double yawAngle = 0.0;
    public static Constants.Alliance alliance = null;

    public static void update(Pose pose, Constants.Alliance autoAlliance) {
        currentPose = pose;
        yawAngle = pose.getHeading();
        alliance = autoAlliance;
    }
}
