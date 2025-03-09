// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoPathPlanner {
    private final CommandSwerveDrivetrain swerveDrive;

    public AutoPathPlanner(CommandSwerveDrivetrain swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public Command followDynamicPath(Pose2d targetPose) {
        // Generate a path dynamically
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(3.0, 2.0), // Max velocity & acceleration
                new PathPoint(swerveDrive.getPose().getTranslation(), swerveDrive.getPose().getRotation()), // Start
                new PathPoint(targetPose.getTranslation(), targetPose.getRotation()) // End
        );

        // Path following command
        return new PPSwerveControllerCommand(
                trajectory,
                swerveDrive::getPose,
                swerveDrive.getKinematics(),
                new PIDController(1.0, 0, 0), // X controller
                new PIDController(1.0, 0, 0), // Y controller
                new PIDController(1.0, 0, 0), // Theta controller
                swerveDrive::setModuleStates,
                swerveDrive
        ).andThen(new InstantCommand(() -> swerveDrive.stopModules()));
    }
}



import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPoint;

