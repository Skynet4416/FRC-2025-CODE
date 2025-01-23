package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.Intake;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex upperIntakeSparkFlex;
    private final SparkFlex lowerIntakeSparkFlex;

    private final RelativeEncoder upperMasterIntakeFlexEncoder;

    public IntakeSubsystem() {
        upperIntakeSparkFlex = new SparkFlex(Intake.Motors.UPPER_MASTER_SPARK_FLEX_ID, MotorType.kBrushless);
        lowerIntakeSparkFlex = new SparkFlex(Intake.Motors.LOWER_SLAVE_SPARK_FLEX_ID, MotorType.kBrushless);
        
        upperIntakeSparkFlex.configure(new SparkFlexConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkFlexConfig slaveConfig = new SparkFlexConfig();
        slaveConfig.follow(upperIntakeSparkFlex,true);

        lowerIntakeSparkFlex.configure(slaveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.upperMasterIntakeFlexEncoder = upperIntakeSparkFlex.getEncoder();
    }

    public void moveMotor(double percentage) {
        this.upperIntakeSparkFlex.set(percentage);
    }

    public void stopMotor() {
        this.upperIntakeSparkFlex.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("current percentage", upperIntakeSparkFlex.get());
        SmartDashboard.putNumber("current velocity RPM", upperMasterIntakeFlexEncoder.getVelocity());
    }

}
