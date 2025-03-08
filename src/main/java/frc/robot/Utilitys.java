// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Utilitys {

    public static Pose2d shiftPoseRight(Pose2d originalPose, double distanceInches) {
        // Get current pose components
        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation(); // Rotation2d object
        
        // Compute new coordinates
        double xNew = x + Units.inchesToMeters(distanceInches) * Math.sin(theta.getRadians());
        double yNew = y - Units.inchesToMeters(distanceInches) * Math.cos(theta.getRadians());
    
        // Return the new pose with the same orientation
        return new Pose2d(xNew, yNew, theta);
    }
    public static Pose2d shiftPoseLeft(Pose2d originalPose, double distanceInches) {
        // Get current pose components
        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation(); // Rotation2d object

        // Compute new coordinates (shift left)
        double xNew = x - Units.inchesToMeters(distanceInches) * Math.sin(theta.getRadians());
        double yNew = y + Units.inchesToMeters(distanceInches) * Math.cos(theta.getRadians());

        // Return the new pose with the same orientation
        return new Pose2d(xNew, yNew, theta);
    }


    public static Pose2d getAprilTagPose(int tagID) {
        try {
            // Load the official FRC AprilTag field layout (2024 example)
            AprilTagFieldLayout fieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

            // Get the tag pose
            Optional<Pose2d> tagPose = fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());

            return tagPose.orElse(null); // Return the pose if found, otherwise null
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
    public String bestCamera (PoseEstimate left, PoseEstimate right) {
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;

        if (left == null && right == null) {
            return null;
        } 
        else if (left == null) {
            return "limelight-right";
        }
        else if (right == null) {
            return "limelight-left";
        }
        else {
            if (left.tagCount >0){
            leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount >0){
            rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return "left";
            }
            else {
                return "right";
            }
        }
    }
    public PoseEstimate bestEstimate(PoseEstimate left, PoseEstimate right) {
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;

        if (left == null && right == null) {
            return null;
        } 
        else if (left == null) {
            return right;
        }
        else if (right == null) {
            return left;
        }
        else {
            if (left.tagCount >0){
            leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount >0){
            rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return left;
            }
            else {
                return right;
            }
        }
    }
}