package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import static edu.wpi.first.units.Units.Newton;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Slider extends SubsystemBase {

  private final TalonFXS slider;

  // private final TalonFXSConfiguration sliderConfigs = new
  // TalonFXSConfiguration();

  // private final Slot0Configs pidConfigs;
  // private final MotionMagicConfigs mmConfigs;

  private static DynamicMotionMagicVoltage dynamic;// = new DynamicMotionMagicVoltage(0, 300, 300, 800);
  private static PositionVoltage sController;// = new PositionVoltage(0);
  private static PositionDutyCycle pControllerDuty;
  private static PositionDutyCycle sliderPositionDutyCycle;// = new PositionDutyCycle(0).withSlot(2);

  private boolean fast = true;
  private double requestedPosition = 0.0;
  private double cachedPosition = 0.0;
  private boolean atPosition = false;
  private boolean updatePending = false;
  private final VelocityDutyCycle velocityRequest;// = new VelocityDutyCycle(0).withSlot(0);

  private static double velocitySetpoint = 0.0;

  // Tunable PID constants
  public double kP = 0.05, kI = 0.0, kD = 0.0, kV = 0.25, kS = 0.6;

  // Motion Magic profiles
  public static double fastVel = 300, fastAcc = 600, fastJerk = 2000;
  public static double slowVel = 60, slowAcc = 900, slowJerk = 1800;

  public Slider() {
    slider = new TalonFXS(35, "Canivore2");
    velocityRequest = new VelocityDutyCycle(0).withSlot(0);
    sController = new PositionVoltage(0);
    sliderPositionDutyCycle = new PositionDutyCycle(0).withSlot(0);
    pControllerDuty = new PositionDutyCycle(0).withSlot(2);
    dynamic = new DynamicMotionMagicVoltage(0, 300, 300, 800);

    // lider.getConfigurator().refresh(sliderConfigs);

    if (Constants.debug) {
      ShuffleboardTab tab = Shuffleboard.getTab("Arms");
      tab.add("Wrist", this);
    }

  }

  public void setBrakeMode(NeutralModeValue mode) {
    // slider.getConfigurator().refresh(sliderConfigs);
    // sliderConfigs.MotorOutput.NeutralMode = mode;
    // slider.getConfigurator().apply(sliderConfigs);
  }

  public void setTargetVelocityRPS(double velocityRPS) {
    velocitySetpoint = velocityRPS;
    velocityRequest.Velocity = velocitySetpoint;
    slider.setControl(velocityRequest);
    SmartDashboard.putNumber("SliderRPSCmd", velocitySetpoint);

  }

  public void stop(TalonFXS motor) {
    motor.stopMotor();
  }

  public double getCurrentVelocity(TalonFXS motor) {
    return motor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetVelocity(TalonFXS motor, double targetRPS, double tolerance) {
    return Math.abs(getCurrentVelocity(motor) - targetRPS) < tolerance;
  }

  public void setMM(double distance) {
    double position = (distance * 12.0) / (9.525 * 16.0);
    slider.setControl(pControllerDuty.withPosition(position));

  }

  public double getMM() {
    double distance = cachedPosition * (9.525 * 16.0) / 12.0;

    return distance;

  }

  public void setPos(double position) {
    setPos(position, true);
  }

  public void setPos(double position, boolean fast) {

    //slider.setControl(sliderPositionDutyCycle.withPosition(position));
    
    // this.fast = fast;
    // this.requestedPosition = position;
    // slider.setControl(
    // dynamic
    // .withVelocity(fast ? fastVel : slowVel)
    // .withAcceleration(fast ? fastAcc : slowAcc)
    // .withJerk(fast ? fastJerk : slowJerk)
    // .withPosition(position)
    //);
  }

  public double getPos() {
    return cachedPosition;
  }

  public boolean atPos() {
    return atPosition;
  }

  @Override
  public void periodic() {
    cachedPosition = slider.getPosition().getValueAsDouble();
    atPosition = Math.abs(cachedPosition - requestedPosition) < 1.0;
    if (Constants.debug) {
      SmartDashboard.putNumber("SliderPos", cachedPosition);
      SmartDashboard.putNumber("SliderMMCmd", requestedPosition);
      SmartDashboard.putBoolean("SliderAtPos", atPosition);

    }

  }

}
