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

public class DriveToAmpPath extends Command {
  private CommandSwerveDrivetrain drivetrain;
  Pose2d where;
  int tadId;
  double leftDist, rightDist;
  double shiftDirection;

  /** Creates a new DriveToAmpPath. */
  public DriveToAmpPath( double shiftDirection) {

    //this.drivetrain = drivetrain;
    this.shiftDirection = shiftDirection;

    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
     
  }



  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //LimelightHelpers.LimelightResults resultsLeft = LimelightHelpers.getLatestResults("limelight-left");

    LimelightHelpers.LimelightResults resultsRight = LimelightHelpers.getLatestResults("limelight-right");
    SmartDashboard.putNumber("right: ", resultsRight.botpose_avgdist);
    SmartDashboard.putBoolean("valid", resultsRight.valid);
    tadId = 0;
   
   /*
    if (resultsLeft.valid){
      leftDist = resultsLeft.botpose_avgdist;
    }else {
      leftDist = 999999;
    }
      */
    if (resultsRight.valid){
      rightDist = resultsRight.botpose_avgdist;
      
      tadId = (int) resultsRight.targets_Fiducials[0].fiducialID;
    } else {
      rightDist = 999999;
    }

    if(shiftDirection==-1){
      where  = Utilitys.shiftPoseLeft(Utilitys.getAprilTagPose(tadId),Units.inchesToMeters(5), 0.164285833);
          }
     if (shiftDirection == 1){     
    where = Utilitys.shiftPoseRight(Utilitys.getAprilTagPose(tadId),Units.inchesToMeters(5),0.164285833);
     }
//
  
    SmartDashboard.putNumberArray("Where", new double[] { where.getX(), where.getY(), where.getRotation().getRadians() });

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Alpha",true);

    // Create the constraints to use while pathfinding. The constraints defined in
    // the path will only be used for the path.

     PathConstraints constraints = new PathConstraints(
     1.0, 4.0,
     Units.degreesToRadians(540), Units.degreesToRadians(720));

/* NOT quite ready to drive it

    Command driveit = AutoBuilder.pathfindToPose(where, constraints);
     driveit.schedule();


     */
    

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
