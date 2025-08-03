package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.InitLogger;

public class Wrist extends SubsystemBase implements Sendable {
  private static final String TAG = "Wrist";

  private final TalonFX wrist;
  private final TalonFXConfigurator wristConfigurator;
  private final MotionMagicVoltage motionMagicRequest;
  private final TalonFXConfiguration wristConfigs;
  private final Slot0Configs pidConfigs;

  // State tracking
  private double requestedPosition = 0.0;
  private double cachedPos = 0.0;
  private boolean atPosition = false;
  private double lastFaultLogTime = 0.0;
  private double lastHighTempWarningTime = 0.0;

  public Wrist() {
    wrist = new TalonFX(36, "Canivore2");
    wristConfigurator = wrist.getConfigurator();
    motionMagicRequest = new MotionMagicVoltage(0);
    wristConfigs = new TalonFXConfiguration();
    pidConfigs = wristConfigs.Slot0;

    // PID coefficients for slot 0
    pidConfigs.kP = 0.05;
    pidConfigs.kI = 0.0;
    pidConfigs.kD = 0.0;
    pidConfigs.kV = 0.0;

    // Motion Magic settings
    wristConfigs.MotionMagic.MotionMagicCruiseVelocity = 25.0;
    wristConfigs.MotionMagic.MotionMagicAcceleration = 80.0;

    // Limit switches and neutral mode
    wristConfigs.ClosedLoopGeneral.ContinuousWrap = false;
    wristConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    wristConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    wristConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.0;
    wristConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.1;
    wristConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply initial configuration
    wristConfigurator.apply(wristConfigs);

    // Shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Arms");
    tab.add("Wrist", this);

    InitLogger.logMessage(TAG, InitLogger.Level.INFO, "Constructed and initial config applied. kP=" + pidConfigs.kP);
    DataLogManager.log("[Wrist] Constructor: initial configuration applied.");
  }

  public void setBrakeMode(NeutralModeValue mode) {
    wristConfigurator.refresh(wristConfigs);
    NeutralModeValue previous = wristConfigs.MotorOutput.NeutralMode;
    wristConfigs.MotorOutput.NeutralMode = mode;
    wristConfigurator.apply(wristConfigs);
    InitLogger.logMessage(TAG, InitLogger.Level.INFO,
        String.format("Brake mode changed from %s to %s", previous, mode));
    DataLogManager.log(String.format("[Wrist] setBrakeMode(): %s -> %s", previous, mode));
  }

  public void setPos(double position) {
    InitLogger.logMessage(TAG, InitLogger.Level.INFO, String.format("setPos(): new setpoint = %.3f", position));
    DataLogManager.log(String.format("[Wrist] setPos(): new setpoint = %.3f", position));
    wrist.setControl(motionMagicRequest.withPosition(position));
    requestedPosition = position;
  }

  public void moveIt(double positionOffset) {
    double newSetpoint = getPos() + positionOffset;
    InitLogger.logMessage(TAG, InitLogger.Level.INFO,
        String.format("moveIt(): offset=%.3f, new setpoint=%.3f", positionOffset, newSetpoint));
    DataLogManager.log(String.format("[Wrist] moveIt(): offset = %.3f, new setpoint = %.3f", positionOffset, newSetpoint));
    setPos(newSetpoint);
  }

  public void setSpeed(double speed) {
    double limitedSpeed = MathUtil.clamp(speed, -0.2, 0.2);
    wrist.set(limitedSpeed);
    if (Math.abs(limitedSpeed) > 1e-6) {
      InitLogger.logMessage(TAG, InitLogger.Level.INFO,
          String.format("setSpeed(): commanded speed=%.3f", limitedSpeed));
      DataLogManager.log(String.format("[Wrist] setSpeed(): %.3f", limitedSpeed));
    }
  }

  public double getPos() {
    return cachedPos;
  }

  public boolean atPos() {
    return atPosition;
  }

  @Override
  public void periodic() {
    // Refresh position and determine error/at-target
    cachedPos = wrist.getPosition().refresh().getValueAsDouble();
    double error = requestedPosition - cachedPos;
    atPosition = Math.abs(error) < 0.3;

    // Telemetry
    String periodicMsg = String.format("periodic(): pos=%.3f, setpoint=%.3f, error=%.3f, atTarget=%b",
        cachedPos, requestedPosition, error, atPosition);
    DataLogManager.log("[Wrist] " + periodicMsg);
    InitLogger.logBoolean(TAG, "AtTarget", atPosition);
    if (Math.abs(error) > 0.5) {
      InitLogger.logMessage(TAG, InitLogger.Level.WARN, periodicMsg);
    }

    // High temperature warning (throttled)
    double temp = wrist.getDeviceTemp().refresh().getValueAsDouble();
    if (temp > 60.0) {
      if (Timer.getFPGATimestamp() - lastHighTempWarningTime > 1.0) {
        lastHighTempWarningTime = Timer.getFPGATimestamp();
        String tempMsg = String.format("High Temp = %.2f°C", temp);
        InitLogger.logMessage(TAG, InitLogger.Level.WARN, tempMsg);
        DataLogManager.log("[Wrist] Warning: " + tempMsg);
      }
    }

    // Fault monitoring ~ every 200 ms
    double now = Timer.getFPGATimestamp();
    if (now - lastFaultLogTime >= 0.2) {
      lastFaultLogTime = now;

      StringBuilder faults = new StringBuilder();
      boolean anyFault = false;

      StatusSignal<Boolean> deviceTempFault = wrist.getFault_DeviceTemp().refresh();
      if (Boolean.TRUE.equals(deviceTempFault.getValue())) {
        anyFault = true;
        faults.append("DeviceTemp ");
      }

      StatusSignal<Boolean> procTempFault = wrist.getFault_ProcTemp().refresh();
      if (Boolean.TRUE.equals(procTempFault.getValue())) {
        anyFault = true;
        faults.append("ProcTemp ");
      }

      StatusSignal<Boolean> hardwareFault = wrist.getFault_Hardware().refresh();
      if (Boolean.TRUE.equals(hardwareFault.getValue())) {
        anyFault = true;
        faults.append("Hardware ");
      }

      if (anyFault) {
        String faultMsg = faults.toString().trim();
        InitLogger.logMessage(TAG, InitLogger.Level.ERROR, "Fault(s): " + faultMsg);
        DataLogManager.log("[Wrist] Fault(s): " + faultMsg);
      }
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Wrist");
    builder.addDoubleProperty("Position", this::getPos, null);
    builder.addDoubleProperty("Setpoint", () -> requestedPosition, this::setPos);
    builder.addBooleanProperty("At Setpoint", this::atPos, null);

    builder.addDoubleProperty("kP", () -> pidConfigs.kP, (val) -> {
      if (pidConfigs.kP != val) {
        wristConfigurator.refresh(wristConfigs);
        pidConfigs.kP = val;
        wristConfigurator.apply(wristConfigs);
        InitLogger.logMessage(TAG, InitLogger.Level.INFO, "Tuned kP -> " + val);
      }
    });

    builder.addDoubleProperty("kF", () -> pidConfigs.kV, (val) -> {
      if (pidConfigs.kV != val) {
        wristConfigurator.refresh(wristConfigs);
        pidConfigs.kV = val;
        wristConfigurator.apply(wristConfigs);
        InitLogger.logMessage(TAG, InitLogger.Level.INFO, "Tuned kV -> " + val);
      }
    });
  }
}
