// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.opencv.core.Mat;

import com.ctre.phoenix6.configs.CommutationConfigs;
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

public class Wrist extends SubsystemBase {

  TalonFX wrist;
  
  TalonFXConfigurator wristConfigurator;
  PositionDutyCycle wristController;
  CommutationConfigs commutationConfigs;
  
  TalonFXConfiguration wristConfigs;
  double requestedPosition;
  boolean atPosition;
  
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  public double i,d,ff,aFF;

  /** Creates a new Wrist. */
  public Wrist() {
    wrist = new TalonFX(36);
    wristConfigurator =  wrist.getConfigurator();
    commutationConfigs = new CommutationConfigs();
    commutationConfigs.MotorArrangement = MotorArrangementValue.Minion_JST;
    
    //SmartDashboard.putString("motor", ww
    wristConfigs = new TalonFXConfiguration();
    wristConfigs.Slot0.kP = 0.5;
   // wristConfigs.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    SmartDashboard.putString("motor", wrist.getConnectedMotor().toString());
    wristConfigs.ClosedLoopGeneral.ContinuousWrap = false;
    //wristConfigs.ExternalFeedback.SensorToMechanismRatio = 1.0;
    
 wrist.getConfigurator().apply(wristConfigs);
    
    

          
  
    
    

   
    // wrist.mo
  //  requestedPosition = getPos();
  /*  SmartDashboard.putNumber("P", kP);
    SmartDashboard.putNumber("I", kI);
    SmartDashboard.putNumber("D", kD);
    SmartDashboard.putNumber("MaxVel", maxVel);
    SmartDashboard.putNumber("MaxAcc", maxAcc);
    SmartDashboard.putNumber("Feed Forward", aFF);

    */


    // utilized encoder 360 only
  }

  public void setPos(double position) {
    wristController = new PositionDutyCycle(position);
    wrist.setControl(wristController);
    requestedPosition = position;
    SmartDashboard.putNumber("Request", position);
    
  }

  public double getPos() {
    var pos = wrist.getPosition();
     // var pulseWidth = wrist.getRawPulseWidthPosition().getValueAsDouble();
     // var quad = wrist.getRawQuadraturePosition().getValueAsDouble();
      var rotor = wrist.getRotorPosition().getValueAsDouble();
    SmartDashboard.putNumber("pos", pos.getValueAsDouble());
   // SmartDashboard.putNumber("pulse", pulseWidth);
   // SmartDashboard.putNumber("quad", quad);
    SmartDashboard.putNumber("rotor", rotor);


    return wrist.getRotorPosition().getValueAsDouble();
  }

  public boolean atPos() {
    return atPosition;
  }

  @Override
  public void periodic() {
    







    if (Math.abs( Math.abs(getPos()) - Math.abs(requestedPosition)) <0.5) {
      atPosition = true;
    } else {
      atPosition = false;

    }
    SmartDashboard.putBoolean("Wrist", atPosition);
    

    // This method will be called once per scheduler run
  }
}
