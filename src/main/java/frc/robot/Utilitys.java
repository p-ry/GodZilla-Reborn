// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class Utilitys {

    public static Pose2d shiftPoseRight(Pose2d originalPose, double forwardInches, double rightInches) {
        // Get current pose components
        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation(); // Rotation2d object

        // Compute new coordinates
        // Convert inches to meters (WPILib uses meters)
        double forwardMeters = Units.inchesToMeters(forwardInches);
        double rightMeters = Units.inchesToMeters(rightInches);

        // Calculate new position
        double xNew = x + forwardMeters * Math.cos(theta.getRadians()) + rightMeters * Math.sin(theta.getRadians());
        double yNew = y + forwardMeters * Math.sin(theta.getRadians()) - rightMeters * Math.cos(theta.getRadians());

        // Return the new pose with the same orientation
        return new Pose2d(xNew, yNew, theta);
    }

    public static Pose2d shiftPoseLeft(Pose2d originalPose, double forwardInches, double leftInches) {
        // Get current pose components

        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation(); // Rotation2d object
        double forwardMeters = Units.inchesToMeters(forwardInches);
        double rightMeters = Units.inchesToMeters(leftInches);

        // Compute new coordinates (shift left)
        double xNew = x + forwardMeters * Math.cos(theta.getRadians()) - rightMeters * Math.sin(theta.getRadians());
        double yNew = y + forwardMeters * Math.sin(theta.getRadians()) + rightMeters * Math.cos(theta.getRadians());

        // Return the new pose with the same orientation
        return new Pose2d(xNew, yNew, theta);
    }

    public Command driveToPose(Pose2d targetPose) {

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(targetPose, constraints, 0);
        return pathfindingCommand;
    }

    public static Pose2d getAprilTagPose(int tagID) {
        try {
            // Load the official FRC AprilTag field layout (2024 example)
            // AprilTagFieldLayout fieldLayout =
            // AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

            // Get the tag pose
            Optional<Pose2d> tagPose = Constants.fieldLayout.getTagPose(tagID).map(pose3d -> pose3d.toPose2d());

            return tagPose.orElse(null); // Return the pose if found, otherwise null
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    public String bestCamera(PoseEstimate left, PoseEstimate right) {
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;

        if (left == null && right == null) {
            return null;
        } else if (left == null) {
            return "limelight-right";
        } else if (right == null) {
            return "limelight-left";
        } else {
            if (left.tagCount > 0) {
                leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount > 0) {
                rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return "left";
            } else {
                return "right";
            }
        }
    }

    public PoseEstimate bestEstimate(PoseEstimate left, PoseEstimate right) {
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;

        if (left == null && right == null) {
            return null;
        } else if (left == null) {
            return right;
        } else if (right == null) {
            return left;
        } else {
            if (left.tagCount > 0) {
                leftAmbiguity = left.rawFiducials[0].ambiguity;
            }
            if (right.tagCount > 0) {
                rightAmbiguity = right.rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                return left;
            } else {
                return right;
            }
        }
    }
}