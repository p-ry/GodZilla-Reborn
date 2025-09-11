// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Winch extends SubsystemBase {

  TalonFX winch,cageWheels;
  double speed;
  double requestedPosition;
  boolean debug = false;
  PositionDutyCycle motorPosRequest = new PositionDutyCycle(0);
  VelocityDutyCycle  motorSpdRequest = new VelocityDutyCycle(0);

  /** Creates a new Winch. */
  public Winch() {
    winch = new TalonFX(42, "Canivore");
    cageWheels = new TalonFX(41, "Canivore");
  }
public double getPos() {
    return winch.getPosition().getValueAsDouble();
  }

  public void setPos(double offset) {
    requestedPosition = getPos() + offset;
    cageWheels.setControl(motorSpdRequest.withVelocity(0.1));
    winch.setControl(motorPosRequest.withPosition(requestedPosition));


    if (debug) {
      SmartDashboard.putNumber("ACE Current Pos", getPos());
      SmartDashboard.putNumber("ACE Target Pos", requestedPosition);
    }
  }
  public void SetSpeed(){
    winch.set(speed);
  }
  public double GetSpeed(){
    return winch.getRotorVelocity().getValueAsDouble();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
