// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class BranchPose{
    public static Pose2d shiftPose(Pose2d originalPose, double forwardInches, double rightInches) {
        // Get the original pose components
        double x = originalPose.getX();
        double y = originalPose.getY();
        Rotation2d theta = originalPose.getRotation(); // Rotation2d stores the angle
        
        // Convert inches to meters (WPILib uses meters)
        double forwardMeters = Units.inchesToMeters(forwardInches);
        double rightMeters = Units.inchesToMeters(rightInches);
        
        // Calculate new position
        double xNew = x + forwardMeters * Math.cos(theta.getRadians()) + rightMeters * Math.sin(theta.getRadians());
        double yNew = y + forwardMeters * Math.sin(theta.getRadians()) - rightMeters * Math.cos(theta.getRadians());

        // Return new Pose2d with same rotation
        return new Pose2d(xNew, yNew, theta);
    }

    public static void main(String[] args) {
        Pose2d originalPose = new Pose2d(1.0, 2.0, new Rotation2d(Math.toRadians(45))); // Example Pose2d
        Pose2d newPose = shiftPose(originalPose, 4.0, 3.0); // Move 4 inches forward, 3 inches right

        System.out.println("Original Pose: " + originalPose);
        System.out.println("New Pose (3 inches right, 4 inches forward): " + newPose);
    }
}

/** Add your docs here. */

