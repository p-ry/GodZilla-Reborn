// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Ace;
import frc.robot.subsystems.ArmAssembly;
import frc.robot.subsystems.LowerArm;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.UpperArm;
import frc.robot.subsystems.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveArmFix extends Command {
  /** Creates a new Retract. */

  double position;
  int level;
  LowerArm lowerArm;
  UpperArm upperArm;
  // Ace ace;
  Wrist wrist;
  Slider slider;
  ArmAssembly myArm;
  int shiftDirection;

  boolean algae;
  public static boolean retract;
  public int prevLevel;

  public MoveArmFix(ArmAssembly myArm, int level, int direction) {

    this.level = level;
    this.shiftDirection = direction;

    this.myArm = myArm;
  }

  /** Creates a new MoveArm. */
  public MoveArmFix(LowerArm lowerArm, UpperArm upperArm, Slider slider, Wrist wrist, int level) {
    this.level = level;
    this.lowerArm = lowerArm;
    this.upperArm = upperArm;
    this.wrist = wrist;
    // this.ace= ace;
    this.slider = slider;
    // DriveToAmpPath moveIt;// = new DriveToAmpPath(RobotContainer.drivetrain,
    // shiftDirection);

    // declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = upperArm.getPos();
    prevLevel = myArm.level;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    
    // SmartDashboard.putNumber("Wristpos", wrist.getPos());
    algae = RobotContainer.Algae.getAsBoolean();
    if((prevLevel!=level)&& ((prevLevel==3)||(prevLevel==4))){
      retract =true;
    }

    switch (level) {
      case 0:

        if(retract){
          if (prevLevel==3){
            upperArm.setPos(22+5);
            
          }else {
            upperArm.setPos(33.5+5);
          }
          if(upperArm.atPosition){
            retract=false;
            prevLevel = level;
          }
          
        }else{
        lowerArm.setPos(1.0); // 16.10 load

        // upperArm.dynamic.Velocity = 1;
        // upperArm.dynamic.Acceleration = 1;
        // upperArm.dynamic.Jerk = 50;
        upperArm.setPos(1.0);
        // upperArm.UpperArmRight.setControl(upperArm.dynamic.withPosition(1.0));
        // upperArm.setPos(0.5);//0.0 load
        slider.setPos(0);
        wrist.setPos(0.0);
        }
        // ace.setSpeed(0);

        // System.out.println("home");
        break;
      case 1:
        if (algae) {
          lowerArm.setPos(18.0);
          upperArm.setPos(1.0);
          slider.setPos(-2.0);
          wrist.setPos(0.0);
        } else {
          lowerArm.setPos(19.00);
          upperArm.setPos(3.7);// 0.0 load
          wrist.setPos(0);
          slider.setPos(-0.5);
        }
        // System.out.println("Level 1");
        break;
      case 2:
        if (algae) {
          lowerArm.setPos(22.0);
          upperArm.setPos(10.0);
          slider.setPos(-2.0);
          wrist.setPos(3.0);
        } else {

          lowerArm.setPos(1.0);
          upperArm.setPos(6.0);
          slider.setPos(-10.0);
          wrist.setPos(5.0);
          break;
          /*
          lowerArm.setPos(13.5);
          upperArm.setPos(11.5);
          slider.setPos(-0.5);
          wrist.setPos(6.5);
          */
          // System.out.println("Level 2");
          // m_rob .setSpeed(1);
        }

        break;
      case 3:
        if (algae) {
          lowerArm.setPos(25.0);
          upperArm.setPos(18.0);
          slider.setPos(-2.0);
          wrist.setPos(0.0);
        } else {
          lowerArm.setPos(22);
          upperArm.setPos(22);
          slider.setPos(-0.5);
          wrist.setPos(6.5);
        }

        // System.out.println("Level 3");
        break;

      case 4:
        lowerArm.setPos(28.2);
        upperArm.setPos(33.5);
        slider.setPos(-43.4);
        wrist.setPos(8.8);

        // System.out.println("Level 4");
        break;

      case 5:

        lowerArm.setPos(30.0);
        upperArm.setPos(4.0);
        slider.setPos(-0.50);
        wrist.setPos(3.0);

        // System.out.println("Level 2");
        // m_rob .setSpeed(1);
        break;
      case 6: //Level 1

        lowerArm.setPos(27.30);
        upperArm.setPos(8.9);
        slider.setPos(-0.50);
        wrist.setPos(0.0);
        break;
      case 12:
        wrist.setPos(wrist.getPos() + 1.0);
        break;

        
      default:
        if (algae) {
          lowerArm.setPos(1.0);
          upperArm.setPos(1.0);
          slider.setPos(-2.0);
          wrist.setPos(3.0);
        } else { 
         
          lowerArm.setPos(1.0); // 16.10 load
          upperArm.setPos(1.0);

          // upperArm.dynamic.Velocity = 1;
          // upperArm.dynamic.Acceleration = 1;
          // upperArm.dynamic.Jerk = 50;
          // upperArm.UpperArmLeft.setControl(upperArm.dynamic.withPosition(1.0));
          // upperArm.UpperArmRight.setControl(upperArm.dynamic.withPosition(1.0));
          // upperArm.setPos(0.5);//0.0 load
          slider.setPos(-0.50);
          wrist.setPos(0.0);
        }

    //myArm.shiftDirection=shiftDirection;
   
    
    

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("AtLevel", myArm.isAtLevel(level));
    return myArm.isAtLevel(level);

  }
}
