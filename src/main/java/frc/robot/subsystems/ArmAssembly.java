// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Level;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.InitLogger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utilitys;

public class ArmAssembly extends SubsystemBase {

  public LowerArm lowerArm;;
  public UpperArm upperArm;
  public Slider slider;
  public Wrist wrist;
  public Ace ace;
  public int level;
  
  
  public int prevLevel;

  // public Ace ace;

  /** Creates a new TheArms. */
  public ArmAssembly(Boolean algae, int level) {

    // maximum range with current gearing
    // retract = false;
    // klowerarm = 41.0;
    // kupperarm = 45;
    // kwrist = 13;
    // kslider = -48;

    InitLogger.time("lowerARm",()-> {
          lowerArm = new LowerArm();
    });
    InitLogger.time("upperARm",()-> {
          upperArm = new UpperArm();
    });
    InitLogger.time("slider",()-> {
          slider = new Slider();
    });
    InitLogger.time("wrist",()-> {
          wrist = new Wrist();
    });
    
    this.level = level;
    prevLevel = level;
  }

  @Override
  public void periodic() {
    

  }

  public boolean isAtLevel() {
   // SmartDashboard.putBoolean("Godzilla Be Ready", (lowerArm.atPos() && upperArm.atPos() && slider.atPos() ));
    return lowerArm.atPos() && upperArm.atPos() && slider.atPos() ;

  }
}
