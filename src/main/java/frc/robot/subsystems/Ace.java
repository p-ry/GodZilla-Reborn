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
import frc.robot.InitLogger;
import frc.robot.RobotContainer;

public class Ace extends SubsystemBase {
  private final TalonFX ace = new TalonFX(37, "Canivore2");
  private final PositionDutyCycle motorPosRequest = new PositionDutyCycle(0);
  private final DutyCycleOut motorSpdRequest = new DutyCycleOut(0);

  private final LaserCan funnelSensor = new LaserCan(10);
  private final LaserCan aceSensor = new LaserCan(11);

  private double requestedPosition;
  private double distFunnel = 1000, distAce = 1000;

  public static boolean gotIt = false;
  public static boolean coralPresent = false;
  public static boolean backup = false;
  public static boolean stateChange = false;
  private static final double DEFAULT_DISTANCE = 1000.0;
  private static final double DETECT_THRESHOLD = 70.0;
  private static final double BACKDRIVE_SPEED = -0.45;
  private static final double INTAKE_SPEED = 0.7;
  public static boolean funnelSensorDetected = false;
  public static boolean aceSensorDetected = false;
  private String stateText = "";
  private double lastLogTime = 0.0; // Initialize lastLogTime to 0
  private boolean debug = false;

  public enum CoralIntakeState {
    IDLE,
    SEARCHING,
    BACKDRIVE,
    INTAKE,
    STOPPED,
    COMPLETE
  }

  private CoralIntakeState currentState = CoralIntakeState.IDLE;
  private CoralIntakeState previousState = CoralIntakeState.IDLE;

  public Ace(int level) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    ace.getConfigurator().refresh(config);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 50;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
    if (debug) {
      SmartDashboard.putNumber("Ace Speed", output);
    }
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

    if (debug) {
      SmartDashboard.putNumber("ACE Current Pos", getPos());
      SmartDashboard.putNumber("ACE Target Pos", requestedPosition);
    }
  }

  private void updateLaserDistances() {
    LaserCan.Measurement mFunnel = funnelSensor.getMeasurement();
    LaserCan.Measurement mAce = aceSensor.getMeasurement();

    distFunnel = (mFunnel != null && mFunnel.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
        ? mFunnel.distance_mm
        : DEFAULT_DISTANCE;

    distAce = (mAce != null && mAce.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
        ? mAce.distance_mm
        : DEFAULT_DISTANCE;
    funnelSensorDetected = distFunnel < DETECT_THRESHOLD;
    aceSensorDetected = distAce < DETECT_THRESHOLD;

    if (Timer.getFPGATimestamp() % 0.1 < 0.02) {
      // SmartDashboard.putNumber("Laser Distance Funnel", distFunnel);
      // SmartDashboard.putNumber("Laser Distance Ace", distAce);
    }
  }

  private void handleIdleState() {
    setSpeed(0.9);

    if (!coralPresent && (funnelSensorDetected || aceSensorDetected)) {
      coralPresent = true;
      currentState = CoralIntakeState.SEARCHING;
      setSpeed(0);
      stateChange = true;
    }
  }

  private void handleSearchingState() {
    if (!funnelSensorDetected && !aceSensorDetected) {
      currentState = CoralIntakeState.BACKDRIVE;
      stateChange = true;
    } else if (funnelSensorDetected && !aceSensorDetected) {
      currentState = CoralIntakeState.INTAKE;
      stateChange = true;
    } else if (funnelSensorDetected && aceSensorDetected) {
      currentState = CoralIntakeState.INTAKE;
      stateChange = true;
    } else if (!funnelSensorDetected && aceSensorDetected) {
      currentState = CoralIntakeState.STOPPED;
      stateChange = true;
    }

  }

  private void handleBackdriveState() {
    setSpeed(BACKDRIVE_SPEED);

    // Check for transitions back to other states
    if (funnelSensorDetected && !aceSensorDetected) {
      currentState = CoralIntakeState.INTAKE;
      stateChange = true;
    } else if (funnelSensorDetected && aceSensorDetected) {
      currentState = CoralIntakeState.INTAKE;
      stateChange = true;
    } else if (!funnelSensorDetected && aceSensorDetected) {
      currentState = CoralIntakeState.STOPPED;
      stateChange = true;
    }

  }

  private void handleIntakeState() {
    setSpeed(INTAKE_SPEED);

    // Check for transitions to other states
    if (!funnelSensorDetected && !aceSensorDetected) {
      currentState = CoralIntakeState.BACKDRIVE;
    } else if (!funnelSensorDetected && aceSensorDetected) {
      currentState = CoralIntakeState.STOPPED;
    }
    stateChange = true;
  }

  private void handleStoppedState() {
    setSpeed(0);
    updateLaserDistances();
    if (!funnelSensorDetected && !aceSensorDetected) {
      currentState = CoralIntakeState.BACKDRIVE;
    } else if (funnelSensorDetected && !aceSensorDetected) {
      currentState = CoralIntakeState.INTAKE;
    } else if (funnelSensorDetected && aceSensorDetected) {
      currentState = CoralIntakeState.INTAKE;
    } else if (!funnelSensorDetected && aceSensorDetected) {

      gotIt = true;
      InitLogger.logMessage("Ace", "Got It");
      stateText = previousState.name() + "-->" + currentState.name();
      InitLogger.logMessage("Ace", stateText);
      currentState = CoralIntakeState.COMPLETE;
    }
    stateChange = true;
  }

  private void handleCompleteState() {
    // if (Constants.AutonomousMode) {
    //   Constants.autoLoaded = true;
    //   Constants.AutonomousMode = false;
    // } else {
    //   updateLaserDistances();
    //   if (!funnelSensorDetected && !aceSensorDetected) {
    //     currentState = CoralIntakeState.BACKDRIVE;
    //     stateChange = true;
    //   }
    // }

    // Stay in complete state until reset
    // setSpeed(0);

  }

  // Optional: Method to get current state for debugging
  public CoralIntakeState getCurrentState() {
    return currentState;
  }

  // Optional: Method to manually reset the state machine
  public void resetStateMachine() {
    currentState = CoralIntakeState.IDLE;
    coralPresent = false;
    gotIt = false;
    Constants.autoLoaded = false;
    stateChange = true;

  }

  @Override
  public void periodic() {

    if (debug) {
      SmartDashboard.putString("Ace State", currentState.name());
    }
    if (stateChange) {
      stateChange = false;
      stateText = previousState.name() + "-->" + currentState.name();
      InitLogger.logMessage("Ace", stateText);
      previousState = currentState;

    }
    // double now = Timer.getFPGATimestamp();
    // if (now - lastLogTime >= 0.1) { // log up to twice a second
    // InitLogger.logDouble("Ace", "Speed", getSpeed());
    // lastLogTime = now;
    // }

    if (RobotContainer.loading || Constants.AutonomousMode) {
      updateLaserDistances();

      switch (currentState) {
        case COMPLETE:
          handleCompleteState();
          break;
        case STOPPED:
          handleStoppedState();
          break;
        case INTAKE:
          handleIntakeState();
          break;
        case BACKDRIVE:
          handleBackdriveState();
          break;
        case SEARCHING:
          handleSearchingState();
          break;
        case IDLE:
          handleIdleState();
          break;

      }
    } else {
      // Reset state when not in loading or autonomous mode
      currentState = CoralIntakeState.IDLE;
      // backup = false;

    }
  }
}

// }

// if (RobotContainer.loading || Constants.AutonomousMode) {
// updateLaserDistances();
// if (!coralPresent && (funnelSensorDetected || aceSensorDetected)) {
// // If either sensor detects something, we assume coral is present
// coralPresent = true;
// setSpeed(0);
// }

// if (coralPresent) {
// if (!funnelSensorDetected && !aceSensorDetected) {
// // If neither sensor detects anything, we backdrive the ace
// setSpeed(BACKDRIVE_SPEED);
// } else if (funnelSensorDetected && !aceSensorDetected) {
// // If only the funnel sensor detects, we set the speed to intake speed
// setSpeed(INTAKE_SPEED);
// } else if (funnelSensorDetected && aceSensorDetected) {
// // If both sensors detect, we set the speed to intake speed
// setSpeed(INTAKE_SPEED);
// } else if (!funnelSensorDetected && aceSensorDetected) {
// // If only the ace sensor detects, we stop the ace
// setSpeed(0);
// gotIt = true;
// }
// }
// }
// else {
// backup = false;
// }
