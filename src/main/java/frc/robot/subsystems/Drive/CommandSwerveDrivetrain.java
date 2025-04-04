package frc.robot.subsystems.Drive;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Subsystems.Drive;
import frc.robot.subsystems.Drive.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.subsystems.Vision.LimelightObserver;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, LimelightObserver {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /**
     * Swerve request to apply during field-centric path following
     */
    private final PIDController m_pathXController = new PIDController(Constants.Subsystems.Drive.Position.KP, 0, 0);
    private final PIDController m_pathYController = new PIDController(Constants.Subsystems.Drive.Position.KP, 0, 0);
    private final PIDController m_pathThetaController = new PIDController(Constants.Subsystems.Drive.Rotation.KP, 0, 0);

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
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can
     * access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_pathThetaController.setTolerance(Units.degreesToRadians(1.5));
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can
     * access them through getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN
     *                                2.0.
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
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_pathThetaController.setTolerance(Units.degreesToRadians(1.5));

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices themselves. If they need the devices, they can
     * access them through getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on CAN FD, and 100 Hz on CAN
     *                                  2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation in the form [x, y, theta]ᵀ, with
     *                                  units in meters and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation in the form [x, y, theta]ᵀ, with
     *                                  units in meters and radians
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
        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_pathThetaController.setTolerance(Units.degreesToRadians(1.5));

    }

    /**
     * Returns a command that applies the specified control request to this
     * swerve drivetrain.
     *
     * @param requestSupplier Function returning the request to apply
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

    @Override
    public void periodic() {
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
        SmartDashboard.putNumber("current angle", getGyroRotationInDegrees());
        SmartDashboard.putString("units", this.getModule(0).getDriveMotor().getClosedLoopReference().getUnits());
        SmartDashboard.putNumber("FL delta velcoity",
                this.getModule(0).getDriveMotor().getClosedLoopReference().getValueAsDouble());
        // m_pathThetaController.setP(SmartDashboard.getNumber("KP", 0));
        // m_pathThetaController.setI(SmartDashboard.getNumber("KI", 0));
        // SmartDashboard.putNumber("KP", m_pathThetaController.getP());
        // SmartDashboard.putNumber("KI", m_pathThetaController.getI());

        LimelightHelpers.SetRobotOrientation("", this.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        SmartDashboard.putNumber("drive x error", m_pathXController.getError());
        SmartDashboard.putNumber("drive y error", m_pathYController.getError());
        SmartDashboard.putNumber("drive theta error", Units.radiansToDegrees(m_pathThetaController.getError()));
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

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the
     *                              vision camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this
     * method will continue to apply to future measurements until a subsequent
     * call to {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement in the form [x, y, theta]ᵀ, with
     *                                 units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    @Override
    public void onLimelightDataUpdate(LimelightHelpers.PoseEstimate poseEstimate, Vector<N3> stdDiviation) {
        if (Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) < 720) {
            addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, stdDiviation);
            SmartDashboard.putBoolean("DRIVE VISION REJECTED", false);

        } else {
            SmartDashboard.putBoolean("DRIVE VISION REJECTED", true);
        }
    }

    public double calculateRotation(double angleInRadians) {
        return m_pathThetaController.calculate(getPose().getRotation().getRadians(), angleInRadians);
    }

    // public void configureAutoBuilder() {
    // try {
    // var config = RobotConfig.fromGUISettings();
    // AutoBuilder.configure(
    // () -> getState().Pose, // Supplier of current robot pose
    // this::resetPose, // Consumer for seeding pose against auto
    // () -> getState().Speeds, // Supplier of current robot speeds
    // // Consumer of ChassisSpeeds and feedforwards to drive the robot
    // (speeds, feedforwards) -> setControl(
    // m_pathApplyRobotSpeeds.withSpeeds(speeds)
    // .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
    // .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
    // ),
    // new PPHolonomicDriveController(
    // // PID constants for translation
    // new PIDConstants(10, 0, 0),
    // // PID constants for rotation
    // new PIDConstants(0, 0, 0)
    // ),
    // config,
    // // Assume the path needs to be flipped for Red vs Blue, this is normally the
    // case
    // () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
    // this // Subsystem for requirements
    // );
    // } catch (Exception ex) {
    // DriverStation.reportError("Failed to load PathPlanner config and configure
    // AutoBuilder", ex.getStackTrace());
    // }
    // }

    public double getGyroRotationInDegrees() {
        return this.getPigeon2().getYaw().getValueAsDouble() % 360;
    }

    public boolean atRotationSetpoint() {
        return this.m_pathThetaController.atSetpoint();
    }

    // shaknet ops
    public void followTrajectory(SwerveSample target) {
        // Get the current pose of the robot
        Pose2d pose = super.getState().Pose;

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                target.vx + m_pathXController.calculate(pose.getX(), target.x),
                target.vy + m_pathYController.calculate(pose.getY(), target.y),
                target.omega + m_pathThetaController.calculate(pose.getRotation().getRadians(), target.heading),
                pose.getRotation());

        // Apply the generated speeds
        setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));

    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public void resetOdometry(Pose2d pose2d) {
        if (LimelightHelpers.getTV("")) {
            resetTranslation(LimelightHelpers.getBotPose2d_wpiBlue("").getTranslation());
            resetRotation(Rotation2d.fromDegrees(LimelightHelpers.getBotPose2d_wpiBlue("")
                    .getRotation().getDegrees()));
        } else
            resetPose(pose2d);

    }
}