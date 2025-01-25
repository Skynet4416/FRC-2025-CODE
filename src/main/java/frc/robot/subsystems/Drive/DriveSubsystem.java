package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem implements Subsystem {
        
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeft;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRight;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeft;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRight;
    private final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>[] modules;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final SwerveDrivetrainConstants drivetrainConstants;
    private final TalonFXConfiguration driveInitialConfigs;
    private final TalonFXConfiguration steerInitialConfigs;
    private final CANcoderConfiguration encoderInitialConfigs;

    private static SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constantCreator;

    private static Slot0Configs steerGains;
    private static Slot0Configs driveGains;

    private final SwerveRequest.FieldCentric driveRequest;

    @SuppressWarnings({ "rawtypes", "static-access", "unchecked" })
    public DriveSubsystem() {

        this.driveInitialConfigs = new TalonFXConfiguration();
        this.steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withStatorCurrentLimitEnable(true));

        this.encoderInitialConfigs = new CANcoderConfiguration();
        this.driveGains = new Slot0Configs()
                .withKP(DriveConstants.PID.Drive.kP).withKI(DriveConstants.PID.Drive.kI)
                .withKD(DriveConstants.PID.Drive.kD)
                .withKS(DriveConstants.PID.Drive.kS).withKV(DriveConstants.PID.Drive.kV)
                .withKA(DriveConstants.PID.Drive.kA);

        this.steerGains = new Slot0Configs()
                .withKP(DriveConstants.PID.Steer.kP).withKI(DriveConstants.PID.Steer.kI)
                .withKD(DriveConstants.PID.Steer.kD)
                .withKS(DriveConstants.PID.Steer.kS).withKV(DriveConstants.PID.Steer.kV)
                .withKA(DriveConstants.PID.Steer.kA);

        this.constantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(driveInitialConfigs)
                .withSteerMotorInitialConfigs(steerInitialConfigs)
                .withEncoderInitialConfigs(encoderInitialConfigs)
                .withDriveMotorGains(driveGains)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGearRatio(DriveConstants.Dimensions.kRotorToSensorRatioDrive)
                .withSteerMotorGearRatio(DriveConstants.Dimensions.kRotorToSensorRatioSteer);

        this.drivetrainConstants = new SwerveDrivetrainConstants()
                .withCANBusName("rio")
                .withPigeon2Id(DriveConstants.ID.kPigeon);

        this.frontLeft = constantCreator.createModuleConstants(
                DriveConstants.ID.kFrontLeftDriveMotor,
                DriveConstants.ID.kFrontLeftSteerMotor,
                DriveConstants.ID.kFrontLeftEncoder,
                DriveConstants.Dimensions.kFrontLeftEncoderOffset,
                DriveConstants.Dimensions.kTrackWidthMeters,
                DriveConstants.Dimensions.kWheelbaseMeters,
                false,
                false,
                false);

        this.frontRight = constantCreator.createModuleConstants(
                DriveConstants.ID.kFrontRightDriveMotor,
                DriveConstants.ID.kFrontRightSteerMotor,
                DriveConstants.ID.kFrontRightEncoder,
                DriveConstants.Dimensions.kFrontRightEncoderOffset,
                -DriveConstants.Dimensions.kTrackWidthMeters,
                DriveConstants.Dimensions.kWheelbaseMeters,
                false,
                false,
                false);

        this.backLeft = constantCreator.createModuleConstants(
                DriveConstants.ID.kBackLeftDriveMotor,
                DriveConstants.ID.kBackLeftSteerMotor,
                DriveConstants.ID.kBackLeftEncoder,
                DriveConstants.Dimensions.kBackLeftEncoderOffset,
                DriveConstants.Dimensions.kTrackWidthMeters,
                -DriveConstants.Dimensions.kWheelbaseMeters,
                false,
                false,
                false);

        this.backRight = constantCreator.createModuleConstants(
                DriveConstants.ID.kBackRightDriveMotor,
                DriveConstants.ID.kBackRightSteerMotor,
                DriveConstants.ID.kBackRightEncoder,
                DriveConstants.Dimensions.kBackRightEncoderOffset,
                -DriveConstants.Dimensions.kTrackWidthMeters,
                -DriveConstants.Dimensions.kWheelbaseMeters,
                false,
                false,
                false);
        this.driveRequest = new SwerveRequest.FieldCentric()
                .withDeadband(DriveConstants.Safety.kMaxSpeedMetersPerSecond * DriveConstants.Safety.kDeadbandValue)
                .withRotationalDeadband(DriveConstants.Safety.kMaxAngularVelocityRadiansPerSecond * DriveConstants.Safety.kDeadbandValue)
                .withDriveRequestType(DriveRequestType.Velocity)
                .withSteerRequestType(SteerRequestType.Position);

        this.modules = new SwerveModuleConstants[] { frontLeft, frontRight, backLeft, backRight };

        this.drivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
                TalonFX::new, // CHECK
                TalonFX::new, // CHECK
                CANcoder::new, // CHECK
                drivetrainConstants,
                modules

        );

    }

    public void setModulesStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            this.drivetrain.getModule(i).apply(new ModuleRequest().withState(states[i]));
            ;
        }
    }

    public SwerveRequest.FieldCentric getDriveRequest() {
        return this.driveRequest;
    }

    public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> getDrivetrain() {
        return this.drivetrain;
    }

}
