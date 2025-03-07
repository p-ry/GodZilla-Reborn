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

  // public Ace ace;

  /** Creates a new TheArms. */
  public ArmAssembly(int level) {

   
    klowerarm = 41.0;
    kupperarm = 45;
    kwrist = 13;
    kslider = -48;
    lowerArm = new LowerArm();
    upperArm = new UpperArm();
    slider = new Slider();
    wrist = new Wrist();
    ace = new Ace();
    this.level = level;
    //Shuffleboard.selectTab("Arm");
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

    switch (level) {
      case 0:
        lowerArm.setPos(1.0); // 16.10 load

        upperArm.dynamic.Velocity = 1;
        upperArm.dynamic.Acceleration = 1;
        upperArm.dynamic.Jerk = 50;
        upperArm.UpperArmLeft.setControl(upperArm.dynamic.withPosition(1.0));
        upperArm.UpperArmRight.setControl(upperArm.dynamic.withPosition(1.0));
        //upperArm.setPos(0.5);//0.0 load
         slider.setPos(0);
         wrist.setPos(0.0);
        // ace.setSpeed(0);

        // System.out.println("home");
        break;
      case 1:
       lowerArm.setPos(18.00);
       upperArm.setPos(4.0);// 0.0 load
        wrist.setPos(0);
       slider.setPos(-0.5);
        // System.out.println("Level 1");
        break;
      case 2:
       lowerArm.setPos(13.5);
      upperArm.setPos(11.5);
         slider.setPos(-0.5);
        wrist.setPos(8.0);
        // System.out.println("Level 2");
        // m_rob .setSpeed(1);

        break;
      case 3:
      lowerArm.setPos(22);
      upperArm.setPos(22);
         slider.setPos(-0.5);
        wrist.setPos(8);
      
       

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
       lowerArm.setPos(klowerarm);
       upperArm.setPos(kupperarm);
       slider.setPos(kslider);
       wrist.setPos(kwrist);


       break;
       

       

      case 12:
        wrist.setPos(wrist.getPos() + 1.0);
        break;

      default:
      lowerArm.setPos(1.0); // 16.10 load

      upperArm.dynamic.Velocity = 1;
      upperArm.dynamic.Acceleration = 1;
      upperArm.dynamic.Jerk = 50;
      upperArm.UpperArmLeft.setControl(upperArm.dynamic.withPosition(1.0));
      upperArm.UpperArmRight.setControl(upperArm.dynamic.withPosition(1.0));
      //upperArm.setPos(0.5);//0.0 load
       slider.setPos(-0.50);
       wrist.setPos(0.0);
       

        // System.out.println("home");
        
          }
    
    double lowerArm = SmartDashboard.getNumber("kLowerArm", klowerarm);
    double upperArm = SmartDashboard.getNumber("kUpperArm", kupperarm);
    double slider = SmartDashboard.getNumber("kSlider", kslider);
    double wrist = SmartDashboard.getNumber("kWrist", kwrist);
    
    
    // SmartDashboard.putNumber("Slider",slider.getPos());
    // SmartDashboard.putNumber("Wrist",wrist.getPos());

    // This method will be called once per scheduler run }
    if(lowerArm != klowerarm){
      klowerarm = lowerArm;
      System.out.println("Changed LowerArm");
    }

    if(upperArm != kupperarm){
      kupperarm = upperArm;
    }
    
    if(wrist != kwrist){
      kwrist = wrist;
    }

    if(slider != kslider){
      kslider = slider;
    }

  } 

  public boolean isAtLevel(int level) {
    return lowerArm.atPos() && upperArm.atPos() && slider.atPos() && wrist.atPos();

  }
}
