// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;

public class Wrist extends SubsystemBase implements Sendable {

  TalonFX wrist;

  TalonFXConfigurator wristConfigurator;
  PositionDutyCycle wristController;
  // CommutationConfigs commutationConfigs;

  TalonFXConfiguration wristConfigs;
  double requestedPosition;
  boolean atPosition;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double i, d, ff, aFF;
  private Slot0Configs pidConfigs = new Slot0Configs();

  /** Creates a new Wrist. */
  public Wrist() {

    wrist = new TalonFX(36);
    wristConfigurator = wrist.getConfigurator();

    wristController = new PositionDutyCycle(0);
    wristConfigs = new TalonFXConfiguration();
    pidConfigs = wristConfigs.Slot0;
    pidConfigs.kP = 0.05;
    wristConfigs.ClosedLoopGeneral.ContinuousWrap = false;
    wrist.getConfigurator().apply(wristConfigs);
    // commutationConfigs = new CommutationConfigs();
    // commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;
    // wristConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    // wristConfigs.ExternalFeedback.SensorToMechanismRatio = 1.0;
    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("Wrist", this);

  }

  public void setPos(double position) {

    wrist.setControl(wristController.withPosition(position)); // wristController = new PositionDutyCycle(position);
    requestedPosition = position;

  }

  public void setSpeed(double speed) {
    wrist.set(speed);
  }

  public double getPos() {
    var pos = wrist.getPosition();
    return wrist.getPosition().getValueAsDouble();
  }

  public boolean atPos() {
    return atPosition;
  }

  @Override
  public void periodic() {

    if (Math.abs(Math.abs(getPos()) - Math.abs(requestedPosition)) < 0.5) {
      atPosition = true;
    } else {
      atPosition = false;

    }
   
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Wrist");
    // builder.addDoubleProperty("Velocity RPM", () ->
    // wrist.getVelocity().getValueAsDouble() * 60, null);
    builder.addDoubleProperty("Position", () -> wrist.getPosition().getValueAsDouble(), null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    // builder.addDoubleProperty("Output Voltage", () ->
    // wrist.getMotorVoltage().getValueAsDouble(), null);
    // PID Tuning
    builder.addDoubleProperty("kP", () -> pidConfigs.kP, (val) -> {
      pidConfigs.kP = val;
      wrist.getConfigurator().apply(pidConfigs);
    });
    builder.addDoubleProperty("kI", () -> pidConfigs.kI, (val) -> {
      pidConfigs.kI = val;
      wrist.getConfigurator().apply(pidConfigs);
    });
    builder.addDoubleProperty("kD", () -> pidConfigs.kD, (val) -> {
      pidConfigs.kD = val;
      wrist.getConfigurator().apply(pidConfigs);
    });
    builder.addDoubleProperty("kF", () -> pidConfigs.kV, (val) -> {
      pidConfigs.kV = val;
      wrist.getConfigurator().apply(pidConfigs);
    });
  }
}
