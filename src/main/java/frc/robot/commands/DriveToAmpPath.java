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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToAmpPath extends Command {
  Pose2d where;
  /** Creates a new DriveToAmpPath. */
  public DriveToAmpPath() {
    
    
    
    
    // Use addRequirements() here to declare subsystem dependencies.

  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    where = new Pose2d(14.0, 5.0, new edu.wpi.first.math.geometry.Rotation2d(4.8));
    // Load the path we want to pathfind to and follow

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //PathPlannerPath path = PathPlannerPath.fromPathFile("Alpha",true);
    
    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));


Command driveit = AutoBuilder.pathfindToPose(where, constraints);
driveit.schedule();


    // Since AutoBuilder is configured, we can use it to build pathfinding commands
  // Command driveIt =
  // AutoBuilder.pathfindThenFollowPath(path, constraints);//.schedule();
   // driveIt.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
