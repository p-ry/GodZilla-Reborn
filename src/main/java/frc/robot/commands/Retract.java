// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAssembly;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Retract extends Command {
  /** Creates a new Retract. */
  ArmAssembly myArm;
  int level;
  double startTime;
  

  public Retract(ArmAssembly myArm, int prevLevel) {
    this.myArm = myArm;
    this.level=prevLevel;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (level==3){
      myArm.upperArm.setPos(22+5);
      
    }else {
      myArm.upperArm.setPos(33.5+5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return (myArm.upperArm.atPos() 
    || (Timer.getFPGATimestamp()-startTime>2.0));
  }
}
