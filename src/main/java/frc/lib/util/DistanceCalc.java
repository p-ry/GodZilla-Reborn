// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import frc.robot.Utilitys;
//import frc.lib.util.AprilTags;

/** Add your docs here. */
public class DistanceCalc {

public static double getDistance(Pose2d botPose, int tagID) {
    AprilTags aprilTags=new AprilTags();
        double distance;
        Pose2d tagPose;
        tagPose = Utilitys.getAprilTagPose(tagID);
        distance = botPose.getTranslation().getDistance(tagPose.getTranslation());
       
        SmartDashboard.putNumber("passDistance", distance);
        return distance;
    }
    public static double getAngle(Pose2d botPose, int tagID,double distance){
         AprilTags aprilTags=new AprilTags();
        double botZ, tagZ,zDiff,angle;
        Pose3d tagPose;
        tagPose = new Pose3d(Utilitys.getAprilTagPose(tagID));
        tagZ = tagPose.getZ();
        botZ = Units.inchesToMeters(24.875);
        zDiff= Math.abs(botZ-tagZ);
        angle = Math.toDegrees(Math.atan(zDiff/distance));
        return angle;
    }

    






}
