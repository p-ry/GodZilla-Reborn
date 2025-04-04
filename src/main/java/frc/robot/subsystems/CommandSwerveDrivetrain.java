package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

//import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Constants;
import com.ctre.phoenix6.swerve.SwerveModule;

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
import edu.wpi.first.math.util.Units;
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
import frc.robot.Utilitys;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public SwerveDrivePoseEstimator m_poseEstimator;
    public LimelightHelpers.PoseEstimate mt2;
    public LimelightHelpers.PoseEstimate leftPose;
    public LimelightHelpers.PoseEstimate rightPose;
    public LimelightHelpers.PoseEstimate[] cameraPoses = new LimelightHelpers.PoseEstimate[2];

    public Pigeon2 gyro;
    // public SwerveDriveOdometry swerveOdometry;
    public PoseEstimate cameraPose;
    // public Pose2d botPose = new Pose2d();
    public Pose2d botPose2d = new Pose2d();
    public Pose3d botPose3d = new Pose3d();
    public PoseEstimate best = new PoseEstimate();
    public Utilitys utils = new Utilitys();
    // private final SwerveModule[] swerveModules = new SwerveModule[] { new
    // SwerveModule(0, 1), new SwerveModule(2, 3),
    // new SwerveModule(4, 5), new SwerveModule(6, 7) };

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    // private final SwerveRequest.SysIdSwerveTranslation
    // m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
    // new SwerveRequest.SysIdSwerveSteerGains();
    // private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
    // new SwerveRequest.SysIdSwerveRotation();

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        gyro = new Pigeon2(0, "Canivore");
        // gyro.setYaw(0);

        // SmartDashboard.putNumber("yaw2", gyro.getYaw().getValueAsDouble());

        getModule(0).getDriveMotor().setPosition(0);
        getModule(1).getDriveMotor().setPosition(0);
        getModule(2).getDriveMotor().setPosition(0);
        getModule(3).getDriveMotor().setPosition(0);

       // botPose2d = new Pose2d();

        // swerveOdometry = new SwerveDriveOdometry(getKinematics(),
        // kBlueAlliancePerspectiveRotation, getModulePositions());
        // if (Utils.isSimulation()) {
        // startSimThread();
        // }

        m_poseEstimator = new SwerveDrivePoseEstimator(Constants.swerveKinematics, getGyroRotation2D(),
                getModulePositions(), getPose(), VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(1.0)));

               

        configureAutoBuilder();
       
        // mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
        /**
         * Sets the operator perspective forward direction.
         *
         * @param rotation The rotation to set as the forward direction.
         */
    }

    /*
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * 
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on
     * CAN FD, and 100 Hz on CAN 2.0.
     * 
     * @param modules Constants for each specific module
     */

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

    public void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () ->  getPose(),/// getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) -> setControl(
                            m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(2., 0, 0),// was 0.5  //was 0.7
                            // kP10
                            // PID constants for rotation
                            new PIDConstants(2, 0, 0)),// was 2.0
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the
                    // case
                    () -> {
                        var alliance =DriverStation.getAlliance();
                        if (alliance.isPresent()){
                            return alliance.get() == DriverStation.Alliance.Red;

                        }
                        return false;
                    },
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

    public void updateCameraPose() {
        boolean doRejectUpdate = false;
        int bestCamera;
        double leftAmbiguity = 0;
        double rightAmbiguity = 0;
        cameraPoses[0] = grabPose("limelight-left");
       cameraPoses[1] = grabPose("limelight-right");

        if (cameraPoses[0] == null && cameraPoses[1] == null) {
            bestCamera = -1;
        } else if (cameraPoses[0] == null) {
            bestCamera = 1;
        } else if (cameraPoses[1] == null) {
            bestCamera = 0;
        } else {
            if (cameraPoses[0].tagCount > 0) {
                leftAmbiguity = cameraPoses[0].rawFiducials[0].ambiguity;
            }
            if (cameraPoses[1].tagCount > 0) {
                rightAmbiguity = cameraPoses[1].rawFiducials[0].ambiguity;
            }
            if (leftAmbiguity < rightAmbiguity) {
                bestCamera = 0;
            } else {
                bestCamera = 1;
            }
        }
        if (bestCamera == -1) {
            doRejectUpdate = true;
        }else {
            if(cameraPoses[bestCamera].tagCount<1){
                doRejectUpdate=true;
            }
        }
        
        // for (int i = 0; i < 2; i++) {

        // doRejectUpdate = false;
        // if (cameraPoses[i] != null) {
        // if (cameraPoses[i].tagCount == 0) {
        // doRejectUpdate = true;
        // }
        // if (cameraPoses[i].pose.getX() < 0) {
        // doRejectUpdate = true;
        // }
        // if (cameraPoses[i].pose.getY() > 7.6) {
        // doRejectUpdate = true;
        // }
        // } else {
        // doRejectUpdate = true;
        // }
        /// SmartDashboard.putNumber("estimated yaw",
        // m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        if (gyro.getAngularVelocityZWorld().getValueAsDouble() > 360) // if our angular velocity is greater
        {
            doRejectUpdate = true;
        }
        SmartDashboard.putBoolean("RejectUpdate", doRejectUpdate);
        if (!doRejectUpdate) {
            SmartDashboard.putNumber("bestcamera",bestCamera);
            SmartDashboard.putNumberArray("CameraPose", new double[] { cameraPoses[bestCamera].pose.getTranslation().getX(), cameraPoses[bestCamera].pose.getTranslation().getY(),
                cameraPoses[bestCamera].pose.getRotation().getRadians() });
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999));
            m_poseEstimator.addVisionMeasurement(
                    cameraPoses[bestCamera].pose,
                    cameraPoses[bestCamera].timestampSeconds);
            // resetOdometry(m_poseEstimator.getEstimatedPosition());
            // swerveOdometry.resetPosition(getGyroRotation2D(), getModulePositions(),
            // m_poseEstimator.getEstimatedPosition());
        }

    }

    

    @Override
    public void periodic() {

        m_poseEstimator.update(getGyroRotation2D(), getModulePositions());
        botPose2d = m_poseEstimator.getEstimatedPosition();
        //SmartDashboard.putNumber("Rotation2D",getGyroRotation2D().getDegrees());

        updateCameraPose();

        //resetOdometry(botPose2d);
        SmartDashboard.putNumberArray("BotPose",
                new double[] { botPose2d.getTranslation().getX(), botPose2d.getTranslation().getY(),
                        botPose2d.getRotation().getRadians() });
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
        // DriverStation.getAlliance().ifPresent(allianceColor -> {
        // setOperatsorPerspectiveForward(
        // allianceColor == Alliance.Red
        // ? kRedAlliancePerspectiveRotation
        // : kBlueAlliancePerspectiveRotation
        // );
        // m_hasAppliedOperatorPerspective = true;
        // });
        // }

    }

    public void resetOdometry(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroRotation2D(),
                getModulePositions(), pose);
    }

    public Rotation2d getGyroscopeRotation() {

        return Rotation2d.fromDegrees(getCompassHeading());//gyro.getYaw().getValueAsDouble());
    }

    public void resetGyro() {
        gyro.setYaw(0);
    }
    public void resetGyro(double heading) {
        gyro.setYaw(heading);
    }

    public void resetGyroToAlliance() {
        gyro.setYaw(DriverStation.getAlliance().get() == Alliance.Red ? 0 : 180);
    }

    public Pose2d getPose() {
        //m_poseEstimator.getEstimatedPosition();
        // return swerveOdometry.getPoseMeters();
        return botPose2d;
    }

    public Pose3d getPose3d() {
        return new Pose3d(getPose());
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {

        getCompassHeading();
        m_poseEstimator.resetPosition(getGyroRotation2D(), getModulePositions(),
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
        SmartDashboard.putNumber("CompassHeading", Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360.0);
    }

    public Rotation2d getGyroRotation2D() {
        SmartDashboard.putNumber("yaw", gyro.getYaw().getValueAsDouble());
        return Rotation2d.fromDegrees(getCompassHeading());//gyro.getYaw().getValueAsDouble());
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

    public SwerveModulePosition[] getModulePositions() {

        /*
         * AI\\
         * SwerveModulePosition[] positions = new
         * SwerveModulePosition[swerveModules.length];
         * for (int i = 0; i < swerveModules.length; i++) {
         * positions[i] = swerveModules[i].getPosition();
         * }
         * return positions;
         */

        // Return the positions of the swerve modules

        return new SwerveModulePosition[] {
                // Replace with actual module positions
                new SwerveModulePosition(getModule(0).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / TunerConstants.kDriveGearRatio,
                        getModule(0).getCurrentState().angle),
                new SwerveModulePosition(getModule(1).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / TunerConstants.kDriveGearRatio,
                        getModule(1).getCurrentState().angle),
                new SwerveModulePosition(getModule(2).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / TunerConstants.kDriveGearRatio,
                        getModule(2).getCurrentState().angle),
                new SwerveModulePosition(getModule(3).getDriveMotor().getPosition().getValueAsDouble()
                        * Constants.wheelCircumference / TunerConstants.kDriveGearRatio,
                        getModule(3).getCurrentState().angle)
        };

    }
}
