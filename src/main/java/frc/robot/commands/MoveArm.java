// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Ace;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.UpperArm;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.commands.FindTag;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArm extends Command {
 int level;
 LowerArm lowerArm;
 UpperArm upperArm;
 //Ace ace;
 Wrist wrist;
 Slider slider;
 ArmAssembly myArm;
 int shiftDirection;
 
  /** Creates a new MoveArm. */
 public MoveArm(ArmAssembly myArm, int level,int direction) {
 
  
  this.level = level;
  this.shiftDirection = direction;
 
  this.myArm = myArm;
}


  /** Creates a new MoveArm. */
  public MoveArm(LowerArm lowerArm, UpperArm upperArm,Slider slider, Wrist wrist,int level) {
    this.level = level;
    this.lowerArm = lowerArm;
    this.upperArm = upperArm;
    this.wrist = wrist;
    //this.ace= ace;
    this.slider = slider;
    //DriveToAmpPath moveIt;// = new DriveToAmpPath(RobotContainer.drivetrain, shiftDirection);
    
    //declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    myArm.level=level;
    //myArm.shiftDirection=shiftDirection;
   
    
   
  }
   
      // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("AtLevel", myArm.isAtLevel());
    return myArm.isAtLevel();
    
  }
}
