package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
//import au.grapplerobotics.LaserCan.Measurement;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Ace extends SubsystemBase {
  private final TalonFX ace = new TalonFX(37, "Canivore2");
  private final PositionDutyCycle motorPosRequest = new PositionDutyCycle(0);
  private final DutyCycleOut motorSpdRequest = new DutyCycleOut(0);

  public final LaserCan funnelSensor = new LaserCan(10);
  public final LaserCan aceSensor = new LaserCan(11);

  private double requestedPosition;
  private double distFunnel = 1000, distAce = 1000;

  public static boolean gotIt = false;
  public static boolean coralPresent = false;
  public static boolean backup = false;

  private static final double DEFAULT_DISTANCE = 1000.0;
  private static final double DETECT_THRESHOLD = 100.0;
  private static final double BACKDRIVE_SPEED = -0.45;
  private static final double INTAKE_SPEED = 0.7;
  public static boolean funnelSensorDetected=false;
  public static boolean aceSensorDetected=false;
  public LaserCan.Measurement mFunnel; //= funnelSensor.getMeasurement();
    public LaserCan.Measurement mAce ;//= aceSensor.getMeasurement();

  public Ace(int level) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    try {
      
      funnelSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
    
      aceSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("Failed to configure laser can sensors");
    }

    Slot0Configs pid = config.Slot0;
    pid.kP = 2.0;
    pid.kI = 0.0;
    pid.kD = 0.0;
    pid.kS = 0.0;
    pid.kV = 0.12;
    pid.kA = 0.01;

    ace.getConfigurator().apply(config);
  }

  public void setBrakeMode(NeutralModeValue mode) {
    MotorOutputConfigs config = new MotorOutputConfigs();
    ace.getConfigurator().refresh(config);
    config.NeutralMode = mode;
    ace.getConfigurator().apply(config);
  }

  public void setSpeed(double speed) {
    double output = Constants.algaeMode.get() ? speed : speed / 2;
    ace.setControl(motorSpdRequest.withOutput(output));
  }

  public double getSpeed() {
    return ace.getRotorVelocity().getValueAsDouble();
  }

  public double getPos() {
    return ace.getPosition().getValueAsDouble();
  }

  public void setPos(double offset) {
    requestedPosition = getPos() + offset;
    ace.setControl(motorPosRequest.withPosition(requestedPosition));
    // SmartDashboard.putNumber("ACE Current Pos", getPos());
    // SmartDashboard.putNumber("ACE Target Pos", requestedPosition);
  }

  private void updateLaserDistances() {
    mFunnel = funnelSensor.getMeasurement();
    mAce = aceSensor.getMeasurement();
    boolean mstatus=(mFunnel!= null);
    // SmartDashboard.putBoolean("mfunel",mstatus);//mFunnel.status== LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
    
    //SmartDashboard.putNumber("mfunel",mFunnel.distance_mm);
    
    distFunnel = (mFunnel != null && mFunnel.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
        ? mFunnel.distance_mm : DEFAULT_DISTANCE;

    distAce = (mAce != null && mAce.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
        ? mAce.distance_mm : DEFAULT_DISTANCE;
    funnelSensorDetected = distFunnel < DETECT_THRESHOLD;
    aceSensorDetected = distAce < DETECT_THRESHOLD;

    if (Timer.getFPGATimestamp() % 0.1 < 0.02) {
      SmartDashboard.putNumber("Laser Distance Funnel", distFunnel);
      SmartDashboard.putNumber("Laser Distance Ace", distAce);
    }
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("loading", RobotContainer.loading);
    if (!RobotContainer.loading) {
      backup = false;

      return;
    }

    updateLaserDistances();
    // SmartDashboard.putBoolean("Funnel Sensor Detected", funnelSensorDetected);
    // SmartDashboard.putBoolean("Ace Sensor Detected", aceSensorDetected);
    // SmartDashboard.putBoolean("Coral Present", coralPresent);

    if (!coralPresent && (funnelSensorDetected || aceSensorDetected)) {
      // If either sensor detects something, we assume coral is present
      coralPresent = true;
      setSpeed(0);
    }

    if (coralPresent) {
      if (!funnelSensorDetected && !aceSensorDetected) {
        // If neither sensor detects anything, we backdrive the ace
        setSpeed(BACKDRIVE_SPEED);
      } else if (funnelSensorDetected && !aceSensorDetected) {
        // If only the funnel sensor detects, we set the speed to intake speed
        setSpeed(INTAKE_SPEED);
      } else if (funnelSensorDetected && aceSensorDetected) {
        // If both sensors detect, we set the speed to intake speed
        setSpeed(INTAKE_SPEED);
      } else if (!funnelSensorDetected && aceSensorDetected) {
        // If only the ace sensor detects, we stop the ace
        setSpeed(0);
        gotIt = true;
      }
    }
  }
}
