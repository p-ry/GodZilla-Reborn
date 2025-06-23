// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import frc.robot.RobotContainer;
import frc.robot.Robot;
import frc.robot.subsystems.ArmAssembly;
import au.grapplerobotics.ConfigurationFailedException;

public class Ace extends SubsystemBase {
  TalonFX ace;
  MotionMagicVoltage controlace;
  TalonFXConfigurator aceConfigurator;
  VelocityVoltage aceController;

  TalonFXConfiguration aceConfigs;
  double requestedPosition;
  boolean atPosition;
  SparkMaxConfig config;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double i, d, ff, aFF;
  public LaserCan laserCan;
  int level;
  double distance;
  LaserCan.Measurement measurement;
  public static boolean gotIt;
  public static boolean coralPresent;
  public static boolean backup = false;

  PositionDutyCycle motorPosRequest;
  DutyCycleOut motorSpdRequest;

  TalonFXConfiguration talonFXConfigs;
  private Slot0Configs pidConfigs;

  /** Creates a new Ace. */
  public Ace(int level) {
    ace = new TalonFX(37, "Canivore2");
    aceConfigs = new TalonFXConfiguration();
    aceConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    aceConfigs.CurrentLimits.SupplyCurrentLimit = 50;
    ace.setNeutralMode(NeutralModeValue.Brake);
    // PID coefficients
    kP = 2.0;
    kI = 0.0;
    kD = 0.000;
    // double kS = .25;
    motorPosRequest = new PositionDutyCycle(0);
    motorSpdRequest = new DutyCycleOut(0);

    pidConfigs = new Slot0Configs();

    pidConfigs = aceConfigs.Slot0;
    pidConfigs.kS = 0.0; // Add 0.25 V output to overcome static friction
    pidConfigs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    pidConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    pidConfigs.kP = kP; // A position error of 2.5 rotations results in 12 V output
    pidConfigs.kI = kI; // no output for integrated error
    pidConfigs.kD = kD; // A velocity error of 1 rps results in 0.1 V output
    ace.getConfigurator().apply(aceConfigs);

    this.level = level;
    laserCan = new LaserCan(10);

    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      // laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 6, 9, 7));
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 8, 8));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
    } catch (ConfigurationFailedException e) {
      e.printStackTrace();
    }
    gotIt = false;
    coralPresent = false;

  }

  public void setSpeed(double speed) {
    if (RobotContainer.Algae.getAsBoolean()) {
      ace.setControl(motorSpdRequest.withOutput(speed));

    } else {
      ace.setControl(motorSpdRequest.withOutput(speed / 2));

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

  public double getPos() {
    return ace.getPosition().getValueAsDouble();
  }

  public void setPos(double position) {
    requestedPosition = getPos() + position;
    ace.setControl(motorPosRequest.withPosition(requestedPosition));
  }

  @Override
  public void periodic() {
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    SmartDashboard.putNumber("LaserDistance", measurement.distance_mm);

    if (RobotContainer.loading) {

      if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        distance = measurement.distance_mm;
        SmartDashboard.putNumber("ValidLASERDistance", distance);

        if ((distance < 100)) {
          coralPresent = true;

        }
        if (coralPresent && distance > 100) {
          setSpeed(0);
          gotIt = true;

        }
      }
      // if (!backup && gotIt) {
      //   backup = true;
      //   setPos(-3.0);// adjust in grip

      //   // if (distance > 100) {

      //   // setSpeed(-0.4);

      //   // } else {
      //   // // setSpeed(0.4);
      //   // // startTime = Timer.getTimestamp();
      //   // // wait(10);

      //   // setSpeed(0.0);
      //   // backup = true;
      //   // setPos(getPos());//adjust in grip
      //   // }
      //   // }
      //   // } else {
      //   // backup= false;
      // } else {
      //   backup = false;
      // }

      // level = RobotContainer.

      // if ((ace.getTorqueCurrent().getValueAsDouble()>35.0) && (level==1)){
      // setSpeed(0);
      // }
      // This method will be called once per scheduler run

    } else {
      backup = false;
    }
  }
}
