// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Utilitys;

/** Add your docs here. */
public class AprilTags {
    public int whichSpeaker() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == DriverStation.Alliance.Red) {
                LimelightHelpers.setCameraPose_RobotSpace("limelight", -0.057, 0.016, Units.inchesToMeters(24.75), 0,
                        30, 0);
                System.out.println("RED");
                return 4;
            } else {
                LimelightHelpers.setCameraPose_RobotSpace("limelight", -0.057, 0.016, Units.inchesToMeters(24.75), 0,
                        30, 180);
                System.out.println("BLUE");
                return 7;
            }

        } else {
            System.out.println("Alliance Error AprilTags.java");
            return 42;
        }
    }

    public double getAngle(Pose2d botPose, int tagID) {
        double botX, botY, tagX, tagY, angle;
        Pose3d tagPose;
        tagPose = new Pose3d(Utilitys.getAprilTagPose(tagID));
        tagX = tagPose.getX();
        tagY = tagPose.getY();
        botX = botPose.getX();
        botY = botPose.getY();
        if (tagX > botX) {
            angle = 180 + Math.atan2(tagY - botY, tagX - botX) * 180 / Math.PI;
        } else {
            angle = Math.atan2(botY - tagY, botX - tagX) * 180 / Math.PI;
        }

        // ***** if (angle>180){angle -=360}

        SmartDashboard.putNumber("speakerAngle", angle);
        return angle;
    }

    public Pose2d pose3Dto2D(Pose3d pose3d) {

        return pose3d.toPose2d();
    }

   /* public Pose3d getTagPose(int tagID) {
        Pose3d tagPose;
        switch (tagID) {
            case 1:
                tagPose = Constants.AprilTags.id1;
                break;
            case 2:
                tagPose = Constants.AprilTags.id2;
                break;
            case 3:
                tagPose = Constants.AprilTags.id3;
                break;
            case 4:
                tagPose = Constants.AprilTags.id4;
                break;
            case 5:
                tagPose = Constants.AprilTags.id5;
                break;
            case 6:
                tagPose = Constants.AprilTags.id6;
                break;
            case 7:
                tagPose = Constants.AprilTags.id7;
                break;
            case 8:
                tagPose = Constants.AprilTags.id8;
                break;
            case 9:
                tagPose = Constants.AprilTags.id9;
                break;
            case 10:
                tagPose = Constants.AprilTags.id10;
                break;
            case 11:
                tagPose = Constants.AprilTags.id11;
                break;
            case 12:
                tagPose = Constants.AprilTags.id12;
                break;
            case 13:
                tagPose = Constants.AprilTags.id13;
                break;
            case 14:
                tagPose = Constants.AprilTags.id14;
                break;
            case 15:
                tagPose = Constants.AprilTags.id15;
                break;
            case 16:
                tagPose = Constants.AprilTags.id16;
                break;
            case 21:
                tagPose = Constants.AprilTags.id21;
                break;
            case 22:
                tagPose = Constants.AprilTags.id22;
                break;
            case 23:
                tagPose = Constants.AprilTags.id23;
                break;
            case 24:
                tagPose = Constants.AprilTags.id24;
                break;
            case 25:
                tagPose = Constants.AprilTags.id25;
                break;
            case 26:
                tagPose = Constants.AprilTags.id26;
                break;
            case 27:
                tagPose = Constants.AprilTags.id27;
                break;
            case 28:
                tagPose = Constants.AprilTags.id28;
                break;
            case 29:
                tagPose = Constants.AprilTags.id29;
                break;
            case 30:
                tagPose = Constants.AprilTags.id30;
                break;
            case 31:
                tagPose = Constants.AprilTags.id31;
                break;
            case 42:
                tagPose = Constants.AprilTags.id42;
                break;
            case 43:
                tagPose = Constants.AprilTags.id43;
                break;
            default:
                tagPose = Constants.AprilTags.id7;
                System.out.println("NO TAG using 7");
                break;
        }
        return tagPose;
    }
*/
}
