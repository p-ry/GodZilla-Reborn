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
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  private boolean allianceSet = false;

  public Robot() {
    CanBridge.runTCP();
  }

  private void ensureRobotContainerInitialized() {
    if (m_robotContainer == null) {
      InitLogger.time("RobotContainerInit", () -> {
        DataLogManager.log("Initializing RobotContainer...");
        m_robotContainer = new RobotContainer();
        DataLogManager.log("Finished RobotContainer init");
      });


      new java.util.Timer().schedule(new java.util.TimerTask() {
        @Override
        public void run() {
         InitLogger.time("schedule Warmps",() -> {
          m_robotContainer.scheduleWarmups();
          });
        }
      }, 1000);
    }
  }

  @Override
  public void robotInit() {
    InitLogger.startLogging();
    InitLogger.logMessage("robot", "RobotInit/Start");
    LiveWindow.disableAllTelemetry();


  }

  @Override
  public void robotPeriodic() {
    ensureRobotContainerInitialized();
    CommandScheduler.getInstance().run();

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        RobotContainer.BlueAlliance = -1;
      } else {
        RobotContainer.BlueAlliance = 1;
      }
    }
  }

  @Override
  public void disabledInit() {
    ensureRobotContainerInitialized();
    //CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledPeriodic() {
    //CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    ensureRobotContainerInitialized();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    ensureRobotContainerInitialized();
    
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    InitLogger.stopLogging();
  }

  @Override
  public void testInit() {
    ensureRobotContainerInitialized();
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
