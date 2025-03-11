// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Slider extends SubsystemBase {
  TalonFXS slider;

  TalonFXSConfigurator sliderConfigurator;
  PositionDutyCycle sliderController;

  TalonFXSConfiguration sliderConfigs;
  double requestedPosition;
  boolean atPosition;
  SparkMaxConfig config;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double i, d, ff, aFF;

  /** Creates a new Slider. */
  public Slider() {

    slider = new TalonFXS(35);
    sliderController = new PositionDutyCycle(0);
     slider.setNeutralMode(NeutralModeValue.Brake);

  }

  public void setPos(double position) {
   // sliderController = new PositionDutyCycle(position);
    slider.setControl(sliderController.withPosition(position));
    requestedPosition = position;
    
   

  }

  public double getPos() {
    return slider.getPosition().getValueAsDouble();

  }

  public boolean atPos() {
    return atPosition;
  }

  @Override
  public void periodic() {
    if (Math.abs(getPos() - requestedPosition) < 0.8) {
      atPosition = true;
    } else {
      atPosition = false;
    }
    // This method will be called once per scheduler run
  }
}
