package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.Utilitys;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.core.CorePigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    // Define the getModulePositions method
    public SwerveModulePosition[] getModulePositions() {
        // Return the positions of the swerve modules
        return new SwerveModulePosition[] {
                // Replace with actual module positions
               new SwerveModulePosition(getModule(0).getDriveMotor().getPosition().getValueAsDouble()/TunerConstants.kDriveGearRatio * Constants.wheelCircumference, getModule(0).getCurrentState().angle),
               new SwerveModulePosition(getModule(1).getDriveMotor().getPosition().getValueAsDouble() /TunerConstants.kDriveGearRatio * Constants.wheelCircumference, getModule(1).getCurrentState().angle),
                new SwerveModulePosition(getModule(2).getDriveMotor().getPosition().getValueAsDouble() /TunerConstants.kDriveGearRatio * Constants.wheelCircumference, getModule(2).getCurrentState().angle),
                new SwerveModulePosition(getModule(3).getDriveMotor().getPosition().getValueAsDouble() /TunerConstants.kDriveGearRatio * Constants.wheelCircumference, getModule(3).getCurrentState().angle)
        };
    }

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public SwerveDrivePoseEstimator m_poseEstimator;
    public LimelightHelpers.PoseEstimate mt2;
    public LimelightHelpers.PoseEstimate leftPose;
    public LimelightHelpers.PoseEstimate rightPose;
    public LimelightHelpers.PoseEstimate[] cameraPoses = new LimelightHelpers.PoseEstimate[2];
    public Pigeon2 gyro;
    public SwerveDriveOdometry swerveOdometry;
    public PoseEstimate cameraPose;
    public Pose2d botPose2d = new Pose2d();
    public Pose3d botPose3d = new Pose3d();
    public PoseEstimate best = new PoseEstimate();
    public Utilitys utils = new Utilitys();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;// m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        gyro = new Pigeon2(0, "Canivore");

        SmartDashboard.putNumber("yaw2", gyro.getYaw().getValueAsDouble());

        swerveOdometry = new SwerveDriveOdometry(getKinematics(), kBlueAlliancePerspectiveRotation,
                getModulePositions());
        if (Utils.isSimulation()) {
            startSimThread();
        }
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        m_poseEstimator = new SwerveDrivePoseEstimator(Constants.swerveKinematics, getGyroYaw(),
                getModulePositions(), getPose());
        configureAutoBuilder();
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(

            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {

        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        gyro = new Pigeon2(0, "Canivore");
        gyro.setYaw(0);
        getModule(0).getDriveMotor().setPosition(0);
        getModule(1).getDriveMotor().setPosition(0);
        getModule(2).getDriveMotor().setPosition(0);
        getModule(3).getDriveMotor().setPosition(0);


        SmartDashboard.putNumber("yaw2", gyro.getYaw().getValueAsDouble());
        swerveOdometry = new SwerveDriveOdometry(getKinematics(), getGyroYaw(),
        getModulePositions());
       // swerveOdometry = new SwerveDriveOdometry(getKinematics(), kBlueAlliancePerspectiveRotation,
       //         getModulePositions());
        if (Utils.isSimulation()) {
            startSimThread();
        }
       

        swerveOdometry.update(getGyroYaw(), getModulePositions());
        m_poseEstimator = new SwerveDrivePoseEstimator(Constants.swerveKinematics, getGyroYaw(),
                getModulePositions(), getPose());
        configureAutoBuilder();
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(2, 0, 0), // kP10
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                    ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    public void updateOdometry() {
        boolean doRejectUpdate = false;
        cameraPoses[0] = grabPose("limelight-left");
        cameraPoses[1] = grabPose("limelight-right");
        for (int i = 0; i < 2; i++) {

            doRejectUpdate = false;
            if (cameraPoses[i] != null) {
                if (cameraPoses[i].tagCount == 0) {
                    doRejectUpdate = true;
                }
                if (cameraPoses[i].pose.getX() < 0) {
                    doRejectUpdate = true;
                }
                if (cameraPoses[i].pose.getY() > 7.6) {
                    doRejectUpdate = true;
                }
            } else {
                doRejectUpdate = true;
            }
           /// SmartDashboard.putNumber("estimated yaw",
                    //m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
            if (gyro.getAngularVelocityZWorld().getValueAsDouble() > 360) // if our angular velocity is greater
            {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        cameraPoses[i].pose,
                        cameraPoses[i].timestampSeconds);
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                        m_poseEstimator.getEstimatedPosition());
            }

        }
        /*
         * LimelightHelpers.PoseEstimate limelightMeasurement =
         * LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
         * if (limelightMeasurement.tagCount >= 2) {
         * m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7,
         * .9999999));
         * m_poseEstimator.addVisionMeasurement(
         * limelightMeasurement.pose,
         * limelightMeasurement.timestampSeconds);
         * }
         * 
         */

    }

    @Override
    public void periodic() {
       
        SmartDashboard.putNumber("Heading", getCompassHeading());
        SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());


        //swerveOdometry.update(getGyroYaw(), getModulePositions());

        m_poseEstimator.update(
           gyro.getRotation2d(),
           getModulePositions());
       updateOdometry();
        botPose2d = getPose();
        botPose3d = getPose3d();
        SmartDashboard.putNumberArray("BotPose",
        new double[] { botPose2d.getTranslation().getX(), botPose2d.getTranslation().getY(),
                     botPose2d.getRotation().getRadians() });
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetPose(Pose2d pose) {
        // ************ Major CHANGE */
        mt2 = grabPose("limelight-left");
        if (mt2.tagCount > 0) {
            pose = mt2.pose;
        }
        // ************************************* */
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);

        // differentialDriveOdometry.resetPosition(getGyroYaw(), m_distance, m_distance,
        // pose);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose3d getPose3d() {
        return new Pose3d(getPose());
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    /*
     * public void zeroHeading() {
     * if (Robot.isRedAlliance()) {
     * gyro.setYaw(180);
     * 
     * } else {
     * gyro.setYaw(0);
     * 
     * }
     * 
     * // gyro.setYaw(0);
     * swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),
     * new Pose2d(getPose().getTranslation(), new Rotation2d()));
     * 
     * }
     */

    public double getCompassHeading() {

        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360.0);
    }

    public Rotation2d getGyroYaw() {
        SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public PoseEstimate grabPose(String camera) {
        LimelightHelpers.SetRobotOrientation(camera, gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        // LimelightHelpers.SetRobotOrientation("limelight-left",getGyroYaw().getDegrees(),
        // 0, 0, 0, 0, 0);

        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camera);
        return mt2;

    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
    // Optional<PoseEstimate> leftEstimate = left.update(swerve.getMegaTag2Yaw());
    // Optional<PoseEstimate> rightEstimate = right.update(swerve.getMegaTag2Yaw());
}
