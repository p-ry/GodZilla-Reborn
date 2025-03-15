// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmAssembly;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Extend extends Command {

  ArmAssembly myArm;
  int level;
  double startTime;
  double timeout;
  boolean finished;

  /** Creates a new Extend. */
  public Extend(ArmAssembly myArm, int prevLevel) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.myArm = myArm;
    this.level=prevLevel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getTimestamp();
    finished=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (level==4){
      myArm.slider.setPos(-45.4);
      
    }else if(level==99){
      myArm.slider.setPos(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elaspedTime = Timer.getTimestamp()-startTime;
    SmartDashboard.putBoolean("retractFinished",(myArm.slider.atPos()||(timeout-startTime>2.0)));
    SmartDashboard.putNumber("elaspedRTime", elaspedTime);
    
    
     return (myArm.slider.atPos() 
    || (elaspedTime>2.0));
  }
}
