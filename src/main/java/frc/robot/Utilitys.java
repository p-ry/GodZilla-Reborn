// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.LimelightHelpers.PoseEstimate;

/** Add your docs here. */
public class Utilitys {

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