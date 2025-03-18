// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import au.grapplerobotics.LaserCan; 
import au.grapplerobotics.ConfigurationFailedException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import au.grapplerobotics.CanBridge;



public class Robot extends TimedRobot {
  private LaserCan laserCan;
  private Command m_autonomousCommand;

  public final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  //public PoseEstimate best = new PoseEstimate();

  public Robot() {
    //enableLiveWindowInTest(true);
    
    m_robotContainer = new RobotContainer();
    CanBridge.runTCP();
  }


  @Override
  public void robotInit() {
    FollowPathCommand.warmupCommand().schedule();
    Pathfinding.setPathfinder(new LocalADStar());
    //RobotContainer.drivetrain.gyro.setYaw(0);
      var alliance = DriverStation.getAlliance();
    
   // laserCan = new LaserCan(10);
    //m_robotContainer.drivetrain.gyro.setYaw(0);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    /* try {
      //laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      //laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    */
   // if (kUseLimelight) {
    //  var driveState = m_robotContainer.drivetrain.getState();
     // double headingDeg = driveState.Pose.getRotation().getDegrees();
     // double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      /*LimelightHelpers.SetRobotOrientation("limelight-left", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      LimelightHelpers.SetRobotOrientation("limelight-right", headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");
      Utilitys utils = new Utilitys();
      best = utils.bestEstimate(llMeasurement, llMeasurement2);
      //LimelightHelpers.Se
      if (best != null && best.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
        
        m_robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
      }*/
    //}
  }

  


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
   // Utilitys.addLimelightVisionMeasurements("limelight-left");
    //Utilitys.addLimelightVisionMeasurements("limelight-right");
   // LaserCan.Measurement measurement = laserCan.getMeasurement();
   // System.out.println("Distance: " + measurement.distance_mm);
    //if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      //System.out.println("The target is " + measurement.distance_mm + "mm away!");
    //} else {
      //System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    //}

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */
  
  
  
  
    




    
  }

  @Override
  public void disabledInit() {
    
   
   /*
      if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        
         * m_robotContainer.s_Candle.setFireProperties(0.7, 0.5, 0.4, 0.2);
         * m_robotContainer.s_Candle.fireLEDs();
        
       // m_robotContainer.s_Candle.setColourProperties(255, 0, 0, 0.75);
       // m_robotContainer.s_Candle.colorLEDs();
      } else if (alliance.get() == DriverStation.Alliance.Blue) {
        
         * m_robotContainer.s_Candle.setFireProperties(0.7, 0.5, 0.4, 0.2);
         * m_robotContainer.s_Candle.fireLEDs();
         
       // m_robotContainer.s_Candle.setColourProperties(0, 0, 255, 0.75);
       // m_robotContainer.s_Candle.colorLEDs();
      }
    }
    */
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.drivetrain.updateCameraPose();
    
    // why keep reseting heading to 0 in disabled  what happens when you rotate the robot
    //RobotContainer.drivetrain.setHeading(new Rotation2d(0));
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
// major Change 3/15/2025  
// why am i resetting the gyro when auto starts up?

   // m_robotContainer.resetGyro();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
   //Utilitys.addLimelightVisionMeasurements("limelight-left");
    //Utilitys.addLimelightVisionMeasurements("limelight-right");
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
