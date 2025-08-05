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
  
  

  public static boolean retract;
  public int prevLevel;
  private double L1 = 0.4962; // meters
    private double L2 = 0.6969;
  double lowerGearRatio = 16*8;
  double upperGearRatio = 25*4;

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


  
public void setJointAngles(double shoulderDeg, double elbowDeg,double sliderPos) {
      //double currentShoulder = getShoulderAngleDeg();
      //double currentElbow = getElbowAngleDeg();

      



wrist.setPos(8);
lowerArm.setPos((shoulderDeg/360)*lowerGearRatio,true);
upperArm.setPos((elbowDeg/360)*upperGearRatio,true);
slider.setPos(sliderPos*8.1/100, true);
//System.out.println("Lower Gear Ratio: " + (shoulderDeg / 360) * lowerGearRatio + ", Upper Gear Ratio: " + (elbowDeg / 360) * upperGearRatio);
SmartDashboard.putNumber("lowerARM!!!!", (shoulderDeg/360)*lowerGearRatio);

}

public void setJointVelocities(double shoulderVelDegPerSec, double elbowVelDegPerSec, double sliderRPS) {
// Convert degrees/sec to rotations/sec, then to motor units per 100ms
double shoulderRPS = shoulderVelDegPerSec / 360.0;
double elbowRPS = elbowVelDegPerSec / 360.0;


// Slider: convert meters/sec to encoder units/sec (assumes 8.1 revs per 100cm => 0.081 revs/cm => 0.81 revs/m)
//double sliderRPS = (sliderVelMPerSec * 8.1); // meters/sec → rev/sec


// Send to motor controllers (ControlMode.Velocity expects units per 100ms)
lowerArm.setTargetVelocityRPS(shoulderRPS);
upperArm.setTargetVelocityRPS(elbowRPS);
slider.setTargetVelocityRPS(sliderRPS);

SmartDashboard.putNumber("ShoulderVelUnits", shoulderRPS);
SmartDashboard.putNumber("ElbowVelUnits", elbowRPS);
SmartDashboard.putNumber("SliderVelUnits", sliderRPS);
}



public void moveToXY(double x, double y) {
  double dx = x;
  double dy = y;
  double dist = Math.sqrt(dx * dx + dy * dy);

  // Check if reachable
  if (dist > L1 + L2 || dist < Math.abs(L1 - L2))
    return;

  double cosTheta2 = (dx * dx + dy * dy - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  double theta2 = Math.acos(cosTheta2);

  double k1 = L1 + L2 * Math.cos(theta2);
  double k2 = L2 * Math.sin(theta2);
  double theta1 = Math.atan2(dy, dx) - Math.atan2(k2, k1);

  lowerArm.setPos(angleToEncoderUnits(theta1)*lowerGearRatio,false);
  upperArm.setPos(angleToEncoderUnits(theta2)*upperGearRatio,false);
  SmartDashboard.putNumber("lowerARM!!!!", theta1*lowerGearRatio);
}


private double angleToEncoderUnits(double radians) {
  return radians * (1 / (2 * Math.PI)); // example for 4096 CPR encoder
}
  @Override
  public void periodic() {
    

  }

  public boolean isAtLevel() {
   // SmartDashboard.putBoolean("Godzilla Be Ready", (lowerArm.atPos() && upperArm.atPos() && slider.atPos() ));
    return lowerArm.atPos() && upperArm.atPos() && slider.atPos() && wrist.atPos();

  }
}
