package frc.robot.subsystems.Intake;

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
    private IntakeState intakeState;
    private IntakeState intendedState;
    private double prevVelocity = 0;

    public IntakeSubsystem() {
        upperIntakeSparkFlex = new SparkFlex(Intake.Motors.UPPER_MASTER_SPARK_FLEX_ID, MotorType.kBrushless);
        lowerIntakeSparkFlex = new SparkFlex(Intake.Motors.LOWER_SLAVE_SPARK_FLEX_ID, MotorType.kBrushless);

        upperIntakeSparkFlex.configure(new SparkFlexConfig(), SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkFlexConfig slaveConfig = new SparkFlexConfig();
        slaveConfig.follow(upperIntakeSparkFlex, true);

        lowerIntakeSparkFlex.configure(slaveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.upperMasterIntakeFlexEncoder = upperIntakeSparkFlex.getEncoder();
    }

    public void moveMotor(double percentage) {
        this.upperIntakeSparkFlex.set(percentage);
        intendedState = percentage > 0 ? IntakeState.FULL : IntakeState.EMPTY;
    }

    public void stopMotor() {
        this.upperIntakeSparkFlex.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("current percentage", upperIntakeSparkFlex.get());
        SmartDashboard.putNumber("current velocity RPM", upperMasterIntakeFlexEncoder.getVelocity());
        SmartDashboard.putNumber("current deceleration", upperMasterIntakeFlexEncoder.getVelocity() - prevVelocity);

        if (intendedState != intakeState) {
            switch (intendedState) {
                case FULL ->
                        intakeState = upperMasterIntakeFlexEncoder.getVelocity() - prevVelocity > Intake.Physical.DECELERATION_THRESHOLD ? IntakeState.FULL : intakeState;
                case EMPTY ->
                        intakeState = upperMasterIntakeFlexEncoder.getVelocity() - prevVelocity > -Intake.Physical.DECELERATION_THRESHOLD ? IntakeState.EMPTY : intakeState;
            }
        }

        prevVelocity = upperMasterIntakeFlexEncoder.getVelocity();
    }

    public IntakeState getState() {
        return intakeState;
    }

}
