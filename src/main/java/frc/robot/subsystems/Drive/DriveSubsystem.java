package frc.robot.subsystems.Drive;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem implements Subsystem {
    private final SwerveModuleConstants frontLeft;
    private final SwerveModuleConstants frontRight;
    private final SwerveModuleConstants backLeft;
    private final SwerveModuleConstants backRight;
    private final SwerveModuleConstants[] modules;
    private final SwerveDrivetrain drivetrain;
    private final SwerveDrivetrainConstants drivetrainConstants;
    private final TalonFXConfiguration driveInitialConfigs;
    private final TalonFXConfiguration steerInitialConfigs;
        
    private static SwerveModuleConstantsFactory constantCreator;

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
                .withStatorCurrentLimitEnable(true)
        );
        this.driveGains = new Slot0Configs()
            .withKP(DriveConstants.PID.Drive.kP).withKI(DriveConstants.PID.Drive.kI).withKD(DriveConstants.PID.Drive.kD)
            .withKS(DriveConstants.PID.Drive.kS).withKV(DriveConstants.PID.Drive.kV).withKA(DriveConstants.PID.Drive.kA);
        
        this.steerGains = new Slot0Configs()
            .withKP(DriveConstants.PID.Steer.kP).withKI(DriveConstants.PID.Steer.kI).withKD(DriveConstants.PID.Steer.kD)
            .withKS(DriveConstants.PID.Steer.kS).withKV(DriveConstants.PID.Steer.kV).withKA(DriveConstants.PID.Steer.kA);
            
        this.constantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
            .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs);

        this.drivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName("*") // * for any
            .withPigeon2Id(DriveConstants.ID.kNavX); //CHECK but why pigeon? cant navX? FUUUUCK 
        
        this.frontLeft = constantCreator.createModuleConstants(
            DriveConstants.ID.kFrontLeftDriveMotor, 
            DriveConstants.ID.kFrontLeftSteerMotor, 
            DriveConstants.ID.kFrontLeftEncoder, 
            0, //CHECK
            DriveConstants.Dimensions.kTrackWidthMeters, 
            DriveConstants.Dimensions.kWheelbaseMeters, 
            false, 
            false, 
            false
        );

        this.frontRight = constantCreator.createModuleConstants(
            DriveConstants.ID.kFrontRightDriveMotor, 
            DriveConstants.ID.kFrontRightSteerMotor, 
            DriveConstants.ID.kFrontRightEncoder, 
            0, //CHECK
            -DriveConstants.Dimensions.kTrackWidthMeters, 
            DriveConstants.Dimensions.kWheelbaseMeters, 
            false, 
            false, 
            false
        );

        this.backLeft = constantCreator.createModuleConstants(
            DriveConstants.ID.kBackLeftDriveMotor, 
            DriveConstants.ID.kBackLeftSteerMotor, 
            DriveConstants.ID.kBackLeftEncoder, 
            0, //CHECK
            DriveConstants.Dimensions.kTrackWidthMeters, 
            -DriveConstants.Dimensions.kWheelbaseMeters, 
            false, 
            false, 
            false
        );

        this.backRight = constantCreator.createModuleConstants(
            DriveConstants.ID.kBackRightDriveMotor, 
            DriveConstants.ID.kBackRightSteerMotor, 
            DriveConstants.ID.kBackRightEncoder, 
            0, //CHECK
            -DriveConstants.Dimensions.kTrackWidthMeters, 
            -DriveConstants.Dimensions.kWheelbaseMeters, 
            false, 
            false, 
            false
        );
        this.driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.Safety.kMaxSpeedMetersPerSecond * 0.1).withRotationalDeadband(DriveConstants.Safety.kMaxAngularVelocityRadiansPerSecond * 0.1) //CHECK
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withSteerRequestType(SteerRequestType.Position);
     
        this.modules = new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};

        
        this.drivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
            TalonFX::new, //CHECK
            TalonFX::new, //CHECK
            CANcoder::new, //CHECK
            drivetrainConstants,
            modules
            
        );


    }

    public void setModulesStates(SwerveModuleState[] states) {
        for (int i = 0; i < states.length; i++) {
            this.drivetrain.getModule(i).apply(new ModuleRequest().withState(states[i]));;
        }
    }

    public SwerveRequest.FieldCentric getDriveRequest() { return this.driveRequest; }

    public SwerveDrivetrain<TalonFX, TalonFX, CANcoder> getDrivetrain() { return this.drivetrain; }


}
