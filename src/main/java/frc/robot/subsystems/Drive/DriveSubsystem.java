package frc.robot.subsystems.Drive;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.PID.Drive;

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

        this.modules = new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};

        
        this.drivetrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
            TalonFX::new, //CHECK
            TalonFX::new, //CHECK
            CANcoder::new, //CHECK
            drivetrainConstants,
            modules
            
        );


    
    }

    public double getHeading() {
        return drivetrain.getPigeon2().getYaw().getValueAsDouble();
    }

    /**
     * Returns the field oriented corrected velocity for a target velocity
    * 
    * @param targetVelocityX
    *                        The target X velocity (Meters Per Second)
    * @param targetVelocityY
    *                        The target Y velocity (Meters Per Second)
    */
    public double getVelocityFieldOriented_X(double targetVelocityX, double targetVelocityY) {
        double offsetAngle = Rotation2d.fromDegrees(-getHeading()).getDegrees() - DriveConstants.Dimensions.fieldHeadingOffset;
        double corrected_velocity = targetVelocityX * Math.cos(Math.toRadians(offsetAngle))
                    - targetVelocityY * Math.sin(Math.toRadians(offsetAngle));
        return corrected_velocity;
    }

    /**
    * Returns the field oriented corrected velocity for a target velocity
    * 
    * @param targetVelocityX
    *                        The target X velocity (Meters Per Second)
    * @param targetVelocityY
    *                        The target Y velocity (Meters Per Second)
    */
    public double getVelocityFieldOriented_Y(double targetVelocityX, double targetVelocityY) {
        double offsetAngle = Rotation2d.fromDegrees(-getHeading()).getDegrees() - DriveConstants.Dimensions.fieldHeadingOffset;
        double corrected_velocity = targetVelocityX * Math.sin(Math.toRadians(offsetAngle))
                    + targetVelocityY * Math.cos(Math.toRadians(offsetAngle));
        return corrected_velocity;
    }


     public void setModules(double xVelocityMps, double yVelocityMps, double rotationVelocityRps, double speedMode) {
        final double slowFactor = 8;
        double speedDivisor = 1 * (1-speedMode) + slowFactor * speedMode;
                
        xVelocityMps /= speedDivisor;
        yVelocityMps /= speedDivisor;
        rotationVelocityRps /= speedDivisor;

        double xVelocityMpsFieldOriented = getVelocityFieldOriented_X(xVelocityMps, yVelocityMps);
        double yVelocityMpsFieldOriented = getVelocityFieldOriented_Y(xVelocityMps, yVelocityMps);

        boolean correctAngle = true;
        if(Math.abs(xVelocityMps) > 0 || Math.abs(yVelocityMps) > 0 || Math.abs(rotationVelocityRps) > 0){
            if (correctAngle) {
                //replace with drivetrain.
                m_targetAngle += Units.radiansToDegrees(rotationVelocityRps)*1.5;
                this.m_swerveSpeeds = new ChassisSpeeds(xVelocityMpsFieldOriented, yVelocityMpsFieldOriented,
                            Math.abs(this.getGyroAngleInRotation2d().getDegrees() - m_targetAngle) > Drive.PID.kThreshold ? -m_pidController.calculate(this.getGyroAngleInRotation2d().getDegrees()) : 0);
                m_pidController.setSetpoint(m_targetAngle-90);
            } else {
                this.m_swerveSpeeds = new ChassisSpeeds(xVelocityMpsFieldOriented, yVelocityMpsFieldOriented,
                            -rotationVelocityRps * 1.2);
            }
        } else {
            this.m_swerveSpeeds = new ChassisSpeeds(0, 0, 0);
        }
        // m_targetAngle = getGyroAngleInRotation2d().getDegrees();

        SwerveModuleState[] target_states = DriveConstants.Dimensions.kinematics.toSwerveModuleStates(this.m_swerveSpeeds);
        for (int i = 0; i < target_states.length; i++) {
            this.drivetrain.getModule(i).apply(new ModuleRequest().withState(target_states[i]));;
        }
    }

    public SwerveDrivetrain getDrivetrain() { return this.drivetrain; }


}
