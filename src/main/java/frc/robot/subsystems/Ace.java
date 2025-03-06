// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import au.grapplerobotics.LaserCan;

public class Ace extends SubsystemBase {
 TalonFXS ace;
  MotionMagicVoltage controlace;
  TalonFXSConfigurator aceConfigurator;
VelocityVoltage aceController;
  
  TalonFXSConfiguration aceConfigs;
  double requestedPosition;
  boolean atPosition;
  SparkMaxConfig config;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double i,d,ff,aFF;
  LaserCan laserCan;

  /** Creates a new Ace. */
  public Ace() {
    ace = new TalonFXS(37);
    
 
  }

  public void setSpeed(double speed) {
    ace.set(speed/2);
    // aceController.setReference(speed,ControlType.kVelocity);
  }

  public void LaserCANStop() {
    if (laserCan == null) 
      setSpeed(0);
  }
  public double getSpeed(){
    return ace.getRotorVelocity().getValueAsDouble();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
