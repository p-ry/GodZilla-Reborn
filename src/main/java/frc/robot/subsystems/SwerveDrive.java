// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.ModulePosition;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class SwerveDrive extends SubsystemBase {
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    public SwerveDrive() {
        // Initialize swerve modules with specified CAN IDs
        swerveModules = new SwerveModule[]{
            new SwerveModule(11, 21), // Front Left
            new SwerveModule(12, 22), // Front Right
            new SwerveModule(13, 23), // Back Left
            new SwerveModule(14, 24)  // Back Right
        };
       SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < swerveModules.length; i++) {
            modulePositions[i] = swerveModules[i].getPosition();
        }
        // Define wheel locations relative to center of robot
        kinematics = new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3),  // Front Left
            new Translation2d(0.3, -0.3), // Front Right
            new Translation2d(-0.3, 0.3), // Back Left
            new Translation2d(-0.3, -0.3) // Back Right
        );

        // Initialize odometry
        odometry = new SwerveDriveOdometry(kinematics, getGyroRotation(),modulePositions);
        
       
    }

    public SwerveModulePosition[] getModulePositions() {
      return new SwerveModulePosition[]{


        swerveModules[0].getPosition(), // Front Left
        swerveModules[1].getPosition(), // Front Right
        swerveModules[2].getPosition(), // Back Left
        swerveModules[3].getPosition()  // Back Right
    };

    }

   
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 3.0); // Limit max speed to 3 m/s

        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public void resetOdometry() {
        odometry.resetPosition(getGyroRotation(),modulePositions,new Pose2d(0, 0, getGyroRotation()));
        
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(CommandSwerveDrivetrain.gyro.getYaw().getValueAsDouble());
    }

    public void stopModules() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
