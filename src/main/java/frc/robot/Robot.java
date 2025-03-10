// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import au.grapplerobotics.LaserCan; 
import au.grapplerobotics.ConfigurationFailedException;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;



public class Robot extends TimedRobot {
  private LaserCan laserCan;
  private Command m_autonomousCommand;

  public final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;
  public PoseEstimate best = new PoseEstimate();

  public Robot() {
    //enableLiveWindowInTest(true);
    
    m_robotContainer = new RobotContainer();
    //CanBridge.runTCP();
  }


  @Override
  public void robotInit() {
    PathfindingCommand.warmupCommand().schedule();
   
  }

  


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    

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
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
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
  public void teleopPeriodic() {}

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
