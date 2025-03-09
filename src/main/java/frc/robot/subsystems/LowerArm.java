// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;

public class LowerArm extends SubsystemBase {

  TalonFX LowerArmLeft;
  TalonFX LowerArmRight;
  Follower LowerArmRightFollower;
  PositionDutyCycle motorRequest;
  double requestedPosition;
  boolean atPosition;

  MotionMagicVoltage controlLower;
  TalonFXConfiguration talonFXConfigsLeft,talonFXConfigsRight;
  TalonFXConfigurator leftConfigurator;
  MotionMagicVoltage leftRequest, rightRequest;
  // PID coefficients
  double kP = 10.0;
  double kI = 0.0;
  double kD = 0.000;
  double maxVel = 200;
  double maxAcc = 600;
  double minVel = 0;
  double kJerk = 1000;
  boolean change = false;
  boolean updatePID = false;

  /** Creates a new LowerArm. */
  public LowerArm() {
    // leftConfigurator
    LowerArmLeft = new TalonFX(31);
    LowerArmRight = new TalonFX(32);
    // talonFXConfigs = new TalonFXConfiguration();
    leftRequest = new MotionMagicVoltage(0);
    rightRequest = new MotionMagicVoltage(0);

    talonFXConfigsLeft = new TalonFXConfiguration();
    talonFXConfigsRight = new TalonFXConfiguration();

    LowerArmLeft.setNeutralMode(NeutralModeValue.Brake);
    LowerArmRight.setNeutralMode(NeutralModeValue.Brake);

    // set slot 0 gains
    var slot0Configs = talonFXConfigsLeft.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = kP; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = kI; // no output for integrated error
    slot0Configs.kD = kD; // A velocity error of 1 rps results in 0.1 V output
    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigsLeft.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = maxVel; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = maxAcc; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
    var slot0ConfigsRight = talonFXConfigsRight.Slot0;
    slot0ConfigsRight.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsRight.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsRight.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsRight.kP = kP; // A position error of 2.5 rotations results in 12 V output
    slot0ConfigsRight.kI = kI; // no output for integrated error
    slot0ConfigsRight.kD = kD; // A velocity error of 1 rps results in 0.1 V output
    // set Motion Magic settings
    var motionMagicConfigsRight = talonFXConfigsRight.MotionMagic;
    motionMagicConfigsRight.MotionMagicCruiseVelocity = maxVel; // Target cruise velocity of 80 rps
    motionMagicConfigsRight.MotionMagicAcceleration = maxAcc; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsRight.MotionMagicJerk = kJerk; // Target jerk of 1600 rps/s/s (0.1 seconds)
     
    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    
    // LowerArmRightFollower = new Follower(31, true);
    // LowerArmRight.setControl(LowerArmRightFollower);
    LowerArmLeft.getConfigurator().apply(talonFXConfigsLeft);
    LowerArmRight.getConfigurator().apply(talonFXConfigsRight);
    LowerArmRight.getConfigurator().apply(motorConfigs);    

    // leftConfigurator = LowerArmLeft.getConfigurator();
    // leftConfigurator.apply(talonFXConfigs);

    requestedPosition = getPos();
   

  }

  public void setPos(double position) {

    LowerArmLeft.setControl(leftRequest.withPosition(position));
    LowerArmRight.setControl(rightRequest.withPosition(position));
    requestedPosition = position;
  }

  public void setSpeed(double speed) {
    
   // LowerArmLeft.set(speed);
    //LowerArmRight.set(speed);
  }

  public double getPos() {
    return LowerArmLeft.getRotorPosition().getValueAsDouble();
  }
  public double getRightPos() {
    return LowerArmRight.getRotorPosition().getValueAsDouble();
  }
 public boolean atPos(TalonFX talon) {
    return Math.abs(talon.getRotorPosition().getValueAsDouble() - requestedPosition) < 0.8;
  }
  public boolean atPos() {
    return atPosition;
  }

  @Override
  public void periodic() {
    //updatePID = SmartDashboard.getBoolean("update", updatePID);
    SmartDashboard.putNumber("LowArm", getPos());
     
    if (atPos(LowerArmLeft) && atPos(LowerArmRight)) {
      atPosition = true;
    } else {
      atPosition = false;
    }
   
    

  }

  // if PID coefficients on SmartDashboard have changed, write new values to
  // controller

  // This method will be called once per scheduler run
}
