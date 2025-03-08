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
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToAmpPath extends Command {
  private CommandSwerveDrivetrain drivetrain;
  Pose2d where;
  double tadId;
  /** Creates a new DriveToAmpPath. */
  public DriveToAmpPath(CommandSwerveDrivetrain drivetrain) {
    
    this.drivetrain = drivetrain;
    
    
    // Use addRequirements() here to declare subsystem dependencies.

  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tadId = drivetrain.mt2.rawFiducials[0].id;

    
    drivetrain.mt2.pose.
       pose.nearest(null);


    //where = new Pose2d();
    //where = new Pose2d(14.0, 5.0, new edu.wpi.first.math.geometry.Rotation2d(4.8));
    // Load the path we want to pathfind to and follow

  }
  

    Pose2d currentPose = // Get the current pose from your odometry system
    Transform2d desiredTransform = new Transform2d(new Translation2d(x, y), new Rotation2d(angle));
    Pose2d newPose = currentPose.plus(desiredTransform);




  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    where = where.transformBy(new Transform2d(3.171, 4.189, Rotation2d.fromDegrees(0)));

    SmartDashboard.putNumber("WhereX",where.getX());
    SmartDashboard.putNumber("WhereY",where.getY());
    
    
    //PathPlannerPath path = PathPlannerPath.fromPathFile("Alpha",true);
    
    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.
   
   // PathConstraints constraints = new PathConstraints(
   //     1.0, 4.0,
   //     Units.degreesToRadians(540), Units.degreesToRadians(720));

//Command driveit = AutoBuilder.pathfindToPose(where, constraints);
//driveit.schedule();


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
    return true;
  }
}
