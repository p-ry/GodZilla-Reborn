package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LowerArm extends SubsystemBase implements Sendable {

  private final TalonFX lowerArmLeft = new TalonFX(31, "Canivore2");
  private final TalonFX lowerArmRight = new TalonFX(32, "Canivore2");

  private final MotionMagicVoltage leftRequest = new MotionMagicVoltage(0);
  private final MotionMagicVoltage rightRequest = new MotionMagicVoltage(0);
  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0).withSlot(0);

  private double requestedPosition;
  private boolean atPosition;
  private boolean fast;
  private static double velocitySetpoint = 0;

  // PID and Motion Magic parameters
  private static double kP = 10.0, kI = 0.0, kD = 0.0, kS = 0.25;
  private static double fastVel = 200, fastAcc = 600, fastJerk = 1000;
  private static double slowVel = 150, slowAcc = 600, slowJerk = 600;

  private final Slot0Configs pidConfigs = new Slot0Configs();
  private final MotionMagicConfigs mmConfigs = new MotionMagicConfigs();
  private static final DynamicMotionMagicVoltage dynamic = new DynamicMotionMagicVoltage(0, fastVel, fastAcc, fastJerk);

  public LowerArm() {
    configureMotor(lowerArmLeft, false);
    configureMotor(lowerArmRight, true);

    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("LowerArm", this);
  }

  private void configureMotor(TalonFX motor, boolean invert) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = 128.0;
    config.Feedback.RotorToSensorRatio = 1.0;

    Slot0Configs slot = config.Slot0;
    slot.kP = kP;
    slot.kI = kI;
    slot.kD = kD;
    slot.kV = 1.0;
    slot.kS = kS;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (invert) config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    config.Voltage.PeakForwardVoltage = 12;
    config.Voltage.PeakReverseVoltage = -12;

    motor.getConfigurator().apply(config);
    motor.setPosition(0);
  }

  public void setTargetVelocityRPS(double velocityRPS) {
    velocitySetpoint = velocityRPS;
    velocityRequest.Velocity = velocitySetpoint;
    lowerArmLeft.setControl(velocityRequest);
    lowerArmRight.setControl(velocityRequest);
  }

  public void stop(TalonFX motor) {
    motor.stopMotor();
  }

  public double getCurrentVelocity(TalonFX motor) {
    return motor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetVelocity(TalonFX motor, double targetRPS, double tolerance) {
    return Math.abs(getCurrentVelocity(motor) - targetRPS) < tolerance;
  }

  public void setPos(double position) {
    this.requestedPosition = position;
    lowerArmLeft.setControl(leftRequest.withPosition(position));
    lowerArmRight.setControl(rightRequest.withPosition(position));
  }

  public void setPos(double position, boolean fast) {
    this.fast = fast;
    requestedPosition = position;

    double vel = fast ? fastVel : slowVel;
    double acc = fast ? fastAcc : slowAcc;
    double jerk = fast ? fastJerk : slowJerk;

    var command = dynamic.withVelocity(vel).withAcceleration(acc).withJerk(jerk).withPosition(position);
    lowerArmLeft.setControl(command);
    lowerArmRight.setControl(command);

    SmartDashboard.putBoolean("Fast", fast);
  }

  public double getPos() {
    return lowerArmLeft.getPosition().getValueAsDouble();
  }

  public double getRightPos() {
    return lowerArmRight.getPosition().getValueAsDouble();
  }

  public boolean atPos(TalonFX talon) {
    return Math.abs(talon.getPosition().getValueAsDouble() - requestedPosition) < 1.0;
  }

  public boolean atPos() {
    return atPosition;
  }

  public void updatePID() {
    lowerArmLeft.getConfigurator().apply(pidConfigs);
    lowerArmRight.getConfigurator().apply(pidConfigs);
  }

  @Override
  public void periodic() {
    boolean moving = Math.abs(velocitySetpoint) >= 0.01;
    lowerArmLeft.setNeutralMode(moving ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    lowerArmRight.setNeutralMode(moving ? NeutralModeValue.Coast : NeutralModeValue.Brake);

    atPosition = atPos(lowerArmLeft) && atPos(lowerArmRight);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("LowerArm");

    builder.addDoubleProperty("Position - Left", this::getPos, null);
    builder.addDoubleProperty("Position - Right", this::getRightPos, null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    builder.publishConstBoolean("Fast", fast);
    builder.publishConstBoolean("AtPosition", atPosition);

    builder.addDoubleProperty("kP", () -> kP, (val) -> kP = val);
    builder.addDoubleProperty("kI", () -> kI, (val) -> kI = val);
    builder.addDoubleProperty("kD", () -> kD, (val) -> kD = val);
    builder.addDoubleProperty("kS", () -> kS, (val) -> kS = val);

    builder.addDoubleProperty("MMVel", () -> slowVel, (val) -> slowVel = val);
    builder.addDoubleProperty("MMAccel", () -> slowAcc, (val) -> slowAcc = val);
    builder.addDoubleProperty("MMJerk", () -> slowJerk, (val) -> slowJerk = val);

    builder.addBooleanProperty("Update", () -> false, (pressed) -> updatePID());
  }
}