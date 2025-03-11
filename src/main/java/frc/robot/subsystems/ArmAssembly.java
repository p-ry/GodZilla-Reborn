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
  ShuffleboardTab tab = Shuffleboard.getTab("Arm");
  public boolean updatePID = false;
  double klowerarm;
  double kupperarm;
  double kwrist;
  double kslider;
  public double shiftDirection;
  boolean algae;
  public static boolean retract;
  public int prevLevel;

  // public Ace ace;

  /** Creates a new TheArms. */
  public ArmAssembly(Boolean algae, int level) {

    retract = false;
    klowerarm = 41.0;
    kupperarm = 45;
    kwrist = 13;
    kslider = -48;
    lowerArm = new LowerArm();
    upperArm = new UpperArm();
    slider = new Slider();
    wrist = new Wrist();
    this.level = level;
    prevLevel = level;
    // this.shiftDirection = shiftDirection;
    // ace = new Ace(level);

    // Shuffleboard.selectTab("Arm");
    SmartDashboard.putBoolean("update", updatePID);
    SmartDashboard.putNumber("kLowerArm", klowerarm);
    SmartDashboard.putBoolean("lowerARM", lowerArm.atPos());
    SmartDashboard.putNumber("kUpperArm", kupperarm);
    SmartDashboard.putNumber("kSlider", kslider);
    SmartDashboard.putNumber("kWrist", kwrist);

    // GenericEntry
    // tab.add("Ace", ace);

  }

  @Override
  public void periodic() {
 SmartDashboard.putNumber("Wristpos", wrist.getPos());
SmartDashboard.putNumber("slider",slider.getPos());
SmartDashboard.putNumber("UpperArm",upperArm.getPos());
SmartDashboard.putNumber("LowerArm", lowerArm.getPos());


  }

  public boolean isAtLevel() {
    return lowerArm.atPos() && upperArm.atPos() && slider.atPos() && wrist.atPos();

  }
}
