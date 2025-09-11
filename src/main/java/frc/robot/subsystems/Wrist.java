package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.InitLogger;

public class Wrist extends SubsystemBase {
  private static final String TAG = "Wrist";

  private final TalonFX wrist;

  private final PositionDutyCycle positionDutyCycle;

  // slew limiter on the commanded position setpoint (units/sec)
  private final SlewRateLimiter positionLimiter;// = new SlewRateLimiter(7.0);

  // State tracking
  private double requestedPosition = 0.0; // desired by caller
  private double cachedPos = 0.0;
  private boolean atPosition = false;
  private double lastFaultLogTime = 0.0;
  private double lastHighTempWarningTime = 0.0;
  private double lastLogTime = 0;

  public Wrist() {
    wrist = new TalonFX(36, "Canivore2");

    positionDutyCycle = new PositionDutyCycle(0);
    positionLimiter = new SlewRateLimiter(7.0);

    if (Constants.debug) { // Shuffleboard grouping
      ShuffleboardTab tab = Shuffleboard.getTab("Arms");
      tab.add("Wrist", this);
    }
    InitLogger.logMessage(TAG, InitLogger.Level.INFO, "Constructed");
  }

  /** Set desired position; internally rate-limited. */
  public void setPos(double position) {
    InitLogger.logMessage(TAG, InitLogger.Level.INFO, String.format("setPos(): new setpoint = %.3f", position));
    requestedPosition = position;
    double limited = positionLimiter.calculate(position);
    InitLogger.logMessage(TAG, InitLogger.Level.INFO, String.format("setPos():    limited setpoint = %.3f", limited));
    wrist.setControl(positionDutyCycle.withPosition(limited));
  }

  public void moveIt(double positionOffset) {
    double newSetpoint = getPos() + positionOffset;
    InitLogger.logMessage(TAG, InitLogger.Level.INFO,
        String.format("moveIt(): offset=%.3f, new setpoint=%.3f", positionOffset, newSetpoint));
    DataLogManager
        .log(String.format("[Wrist] moveIt(): offset = %.3f, new setpoint = %.3f", positionOffset, newSetpoint));
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
    if (!atPosition) {
      double limitedSetpoint = positionLimiter.calculate(requestedPosition);
      wrist.setControl(positionDutyCycle.withPosition(limitedSetpoint));
    }
    // Telemetry

    if (Constants.debug) { // log up to 20Hz
      String periodicMsg = String.format("periodic(): pos=%.3f, setpoint=%.3f, error=%.3f, atTarget=%b",
          cachedPos, requestedPosition, error, atPosition);
      // Note: if InitLogger lacks logDouble helper, replace with appropriate logging
      // call
      InitLogger.logMessage(TAG, InitLogger.Level.INFO, periodicMsg);
      // InitLogger.logMessage("Wrist POS",String.format("pos=%.3f",cachedPos);
      InitLogger.logDouble(TAG, "Position", cachedPos);
      if (Math.abs(error) > 0.5) {
        InitLogger.logMessage(TAG, InitLogger.Level.WARN, periodicMsg);
      }

    }

    InitLogger.logBoolean(TAG, "AtTarget", atPosition);

    // Fault monitoring ~ every 200 ms
    if (Constants.debug) {

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

}
