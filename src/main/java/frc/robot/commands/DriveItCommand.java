// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.nio.file.Path;
import java.nio.file.WatchService;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Utilitys;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.BranchPose;

public class DriveItCommand extends Command {
  private CommandSwerveDrivetrain drivetrain;
  Pose2d where, botPose2d;
  // int tadId;
  double leftDist, rightDist;
  double shiftDirection;
  int[] tagIds = new int[2];
  boolean validTarget = false;
  int tagId = 0;

  // private final CommandSwerveDrivetrain drivetrain;
  // private final double shiftDirection;

  // private final PathPlannerPath path;
  // private final PathConstraints constraints;

  // private final AutoBuilder autoBuilder;

  // private final PathPlannerState startState;
  // private final PathPlannerState endState;

  // private final Pose2d startPose;
  // private final Pose2d endPose;

  /** Creates a new DriveToAmpPath. */
  PathConstraints constraints;
  Command driveIt;
  boolean whereSet;
  boolean rightTree;

  public DriveItCommand(boolean rightTree) {

    // this.drivetrain = drivetrain;
    this.rightTree = rightTree;

    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    driveIt = Utilitys.driveToIt(rightTree);
    if (driveIt != null) {
      driveIt.schedule();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }
  // */

  // Since AutoBuilder is configured, we can use it to build pathfinding commands
  // Command driveIt =
  // AutoBuilder.pathfindThenFollowPath(path, constraints);//.schedule();
  // driveIt.execute();

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (driveIt != null) {

      if (driveIt.isFinished()) {
        return true;
      }
    }

    return false;
  }
}
