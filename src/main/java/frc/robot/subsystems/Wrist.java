package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import edu.wpi.first.math.MathUtil;

public class Wrist extends SubsystemBase implements Sendable {

  private final TalonFX wrist;
  private final TalonFXConfigurator wristConfigurator;
  private final MotionMagicVoltage motionMagicRequest;

  private final TalonFXConfiguration wristConfigs;
  private final Slot0Configs pidConfigs;

  private double requestedPosition = 0;
  private double newPosition = 0;
  private boolean atPosition = false;

  private double cachedPos = 0;

  /** Creates a new Wrist subsystem. */
  public Wrist() {
    wrist = new TalonFX(36, "Canivore2");
    wristConfigurator = wrist.getConfigurator();
    motionMagicRequest = new MotionMagicVoltage(0);
    wristConfigs = new TalonFXConfiguration();
    pidConfigs = wristConfigs.Slot0;

    // PID tuning
    pidConfigs.kP = 0.05;
    pidConfigs.kI = 0.0;
    pidConfigs.kD = 0.0;
    pidConfigs.kV = 0.0;

    // Motion Magic profile
    wristConfigs.MotionMagic.MotionMagicCruiseVelocity = 25.0;
    wristConfigs.MotionMagic.MotionMagicAcceleration = 80.0;

    // Limits and safety
    wristConfigs.ClosedLoopGeneral.ContinuousWrap = false;
    wristConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.0;
    wristConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.1;
    wristConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wristConfigurator.apply(wristConfigs);

    // Shuffleboard grouping
    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("Wrist", this);
  }

  public void setBrakeMode(NeutralModeValue mode) {
    wrist.getConfigurator().refresh(wristConfigs);
    wristConfigs.MotorOutput.NeutralMode = mode;
    wrist.getConfigurator().apply(wristConfigs);
  }

  public void setPos(double position) {
    wrist.setControl(motionMagicRequest.withPosition(position));
    requestedPosition = position;
  }

  public void setSpeed(double speed) {
    double limitedSpeed = MathUtil.clamp(speed, -0.2, 0.2);
    wrist.set(limitedSpeed);
  }

  public double getPos() {
    return cachedPos;
  }

  public boolean atPos() {
    return atPosition;
  }

  public void moveIt(double positionOffset) {
    newPosition = getPos() + positionOffset;
    setPos(newPosition);
  }

  @Override
  public void periodic() {
    cachedPos = wrist.getPosition().getValueAsDouble();
    atPosition = Math.abs(cachedPos - requestedPosition) < 0.3;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Wrist");

    builder.addDoubleProperty("Position", this::getPos, null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);

    builder.addDoubleProperty("kP", () -> pidConfigs.kP, (val) -> {
      if (pidConfigs.kP != val) {
        pidConfigs.kP = val;
        wrist.getConfigurator().apply(pidConfigs);
      }
    });
    builder.addDoubleProperty("kI", () -> pidConfigs.kI, (val) -> {
      if (pidConfigs.kI != val) {
        pidConfigs.kI = val;
        wrist.getConfigurator().apply(pidConfigs);
      }
    });
    builder.addDoubleProperty("kD", () -> pidConfigs.kD, (val) -> {
      if (pidConfigs.kD != val) {
        pidConfigs.kD = val;
        wrist.getConfigurator().apply(pidConfigs);
      }
    });
    builder.addDoubleProperty("kF", () -> pidConfigs.kV, (val) -> {
      if (pidConfigs.kV != val) {
        pidConfigs.kV = val;
        wrist.getConfigurator().apply(pidConfigs);
      }
    });
  }
}
