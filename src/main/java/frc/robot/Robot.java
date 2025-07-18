// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.ConfigurationFailedException;

import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
//import com.pathplanner.lib.pathfinding.LocalGrid; // Ensure LocalGrid is imported

import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {
  // private LaserCan laserCan;
  private Command m_autonomousCommand;

  public RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  private boolean allianceSet = false;
  // public PoseEstimate best = new PoseEstimate();

  public Robot() {
    // enableLiveWindowInTest(true);

    // m_robotContainer = new RobotContainer();
    CanBridge.runTCP();
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();


    new java.util.Timer().schedule(new java.util.TimerTask() {
      @Override
      public void run() {
          System.out.println("[Timer] Scheduling real warmups...");
          m_robotContainer.scheduleWarmups();  // calls both warmup schedules
      }
  }, 1000);  // delay 1 second to be safe

    
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    
    // *** July 12
    var alliance = DriverStation.getAlliance();
    // if (!allianceSet) {
    if (alliance.isPresent()) {
      // allianceSet = true;

      if (alliance.get() == DriverStation.Alliance.Red) {
        RobotContainer.BlueAlliance = -1;
        // RobotContainer.candle.setLEDs(255, 127, 102);

        // m_robotContainer.s_Candle.setColourProperties(255, 0, 0, 0.75);
        // m_robotContainer.s_Candle.colorLEDs();
      } else {
        RobotContainer.BlueAlliance = 1;
        // RobotContainer.candle.setLEDs(255, 127, 102);
        // RobotContainer.candle.animate(new FireAnimation(1, 0.2, 1, 1, 1, false, 0));
        // RobotContainer.candle.fireLEDs(); // Method not defined in CANdle class

        // m_robotContainer.s_Candle.setColourProperties(0, 0, 255, 0.75);
        // m_robotContainer.s_Candle.colorLEDs();

      }
    }
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().run();
    // RobotContainer.drivetrain.gyro.setYaw(0);
  }

  @Override
  public void disabledPeriodic() {

    CommandScheduler.getInstance().run();
   

    // RobotContainer.drivetrain.updateCameraPose();
    // July 13 2025

  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // m_robotContainer.resetGyro();
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    
  }

  @Override
  public void teleopPeriodic() {
    // Utilitys.addLimelightVisionMeasurements("limelight-left");
    // Utilitys.addLimelightVisionMeasurements("limelight-right");
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
