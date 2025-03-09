// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import au.grapplerobotics.LaserCan;
import frc.robot.RobotContainer;
import frc.robot.Robot;
import frc.robot.subsystems.ArmAssembly;

public class Ace extends SubsystemBase {
  TalonFX ace;
  MotionMagicVoltage controlace;
  TalonFXConfigurator aceConfigurator;
  VelocityVoltage aceController;

  TalonFXConfiguration aceConfigs;
  double requestedPosition;
  boolean atPosition;
  
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double i, d, ff, aFF;
  LaserCan laserCan;
  int level;
  double distance;
  LaserCan.Measurement measurement;
  public boolean gotIt;

  /** Creates a new Ace. */
  public Ace(int level) {
    ace = new TalonFX(37);
    aceConfigs = new TalonFXConfiguration();
    aceConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    aceConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    ace.setNeutralMode(NeutralModeValue.Brake);
    this.level = level;
    laserCan = new LaserCan(10);

  }

  public void setSpeed(double speed) {
    if (RobotContainer.Algae.getAsBoolean()) {
      ace.set(speed);
    } else {
      ace.set(speed / 2);
    }
    // aceController.setReference(speed,ControlType.kVelocity);
  }

  public void LaserCANStop() {
    if (laserCan == null)
      setSpeed(0);
  }

  public double getSpeed() {
    return ace.getRotorVelocity().getValueAsDouble();

  }

  @Override
  public void periodic() {
    level = RobotContainer.mArm.level;
   
    if (level == 1) {
      measurement = laserCan.getMeasurement();
      distance = measurement.distance_mm;
      SmartDashboard.putNumber("distance", distance);
      if (!gotIt) {
        setSpeed(0.6);
      }
     
      if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        distance = measurement.distance_mm;
        SmartDashboard.putNumber("distance", distance);
        if (distance > 370) {
          setSpeed(0);
          gotIt = true;
        }     }
      // level = RobotContainer.

      // if ((ace.getTorqueCurrent().getValueAsDouble()>35.0) && (level==1)){
      // setSpeed(0);
      // }
      // This method will be called once per scheduler run
    }
  }
}