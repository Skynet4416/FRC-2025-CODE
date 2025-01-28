package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    private final SwerveDrivetrainConstants drivetrainConstants;
    private final Slot0Configs steerGains;
    private final Slot0Configs driveGains;
    private final TalonFXConfiguration driveInitialConfigs;
    private final TalonFXConfiguration steerInitialConfigs;

    public DriveSubsystem() {
        drivetrainConstants = new SwerveDrivetrainConstants().withCANBusName(getName()).withPigeon2Id(0)
                .withPigeon2Configs(null);
        steerGains = new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
        driveGains = new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

        driveInitialConfigs = new TalonFXConfiguration();
        steerInitialConfigs = new TalonFXConfiguration()
                .withCurrentLimits(
                        new CurrentLimitsConfigs()
                                // Swerve azimuth does not require much torque output, so we can set a
                                // relatively low
                                // stator current limit to help avoid brownouts without impacting performance.
                                .withStatorCurrentLimit(Amps.of(60))
                                .withStatorCurrentLimitEnable(true));
        
    }
}
