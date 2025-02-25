package frc.robot.subsystems.Intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.Intake;
import frc.robot.subsystems.Elevator.ElevatorState;

import java.util.function.Consumer;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkFlex upperIntakeSparkFlex;
    private final SparkFlex lowerIntakeSparkFlex;

    private final RelativeEncoder upperMasterIntakeFlexEncoder;
    private final RelativeEncoder lowerIntakeFlexEncoder;

    private IntakeState intakeState = IntakeState.EMPTY; //TODO: remove this
    private IntakeState intendedState = intakeState;
    private boolean reachedVelocity = false;
    private double prevVelocityUp = 0;
    private double prevVelocityDown = 0;

    public IntakeSubsystem() {
        upperIntakeSparkFlex = new SparkFlex(Intake.Motors.UPPER_MASTER_SPARK_FLEX_ID, MotorType.kBrushless);
        lowerIntakeSparkFlex = new SparkFlex(Intake.Motors.LOWER_SLAVE_SPARK_FLEX_ID, MotorType.kBrushless);
        SparkBaseConfig masterConfig = new SparkFlexConfig().smartCurrentLimit(35).idleMode(SparkBaseConfig.IdleMode.kBrake);
        upperIntakeSparkFlex.configure(masterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkBaseConfig slaveConfig = new SparkFlexConfig().smartCurrentLimit(35).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);
        slaveConfig.follow(upperIntakeSparkFlex);

        lowerIntakeSparkFlex.configure(slaveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.upperMasterIntakeFlexEncoder = upperIntakeSparkFlex.getEncoder();
        this.lowerIntakeFlexEncoder = lowerIntakeSparkFlex.getEncoder();
    }

    public void moveMotor(double percentage) {
        this.upperIntakeSparkFlex.set(percentage);
        intendedState = percentage > 0 ? IntakeState.FULL : IntakeState.EMPTY;
    }

    public void stopMotor() {
        this.moveMotor(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("current percentage", upperIntakeSparkFlex.get());
        SmartDashboard.putNumber("current velocity top RPM", upperMasterIntakeFlexEncoder.getVelocity());
        SmartDashboard.putNumber("current velocity bottom RPM", lowerIntakeFlexEncoder.getVelocity());
        SmartDashboard.putNumber("Current intake", upperIntakeSparkFlex.getOutputCurrent());
        SmartDashboard.putNumber("current deceleration", upperMasterIntakeFlexEncoder.getVelocity() - prevVelocityUp);
        SmartDashboard.putString("Current Intake state", intakeState.toString());
        SmartDashboard.putString("Current Intake intended state", intendedState.toString());
        SmartDashboard.putBoolean("Reached velocity", reachedVelocity);
    }

    public double getIntakeVelocity() {
        return upperMasterIntakeFlexEncoder.getVelocity();
    }

    public IntakeState getState() {
        return intakeState;
    }

    public void setState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

}
