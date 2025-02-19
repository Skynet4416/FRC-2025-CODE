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
    private final Consumer<ElevatorState> setElevatorState;

    public IntakeSubsystem(Consumer<ElevatorState> setElevatorState) {
        upperIntakeSparkFlex = new SparkFlex(Intake.Motors.UPPER_MASTER_SPARK_FLEX_ID, MotorType.kBrushless);
        lowerIntakeSparkFlex = new SparkFlex(Intake.Motors.LOWER_SLAVE_SPARK_FLEX_ID, MotorType.kBrushless);
        SparkBaseConfig masterConfig = new SparkFlexConfig().smartCurrentLimit(30).idleMode(SparkBaseConfig.IdleMode.kBrake);
        upperIntakeSparkFlex.configure(masterConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkBaseConfig slaveConfig = new SparkFlexConfig().smartCurrentLimit(30).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);
        slaveConfig.follow(upperIntakeSparkFlex);

        lowerIntakeSparkFlex.configure(slaveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        this.upperMasterIntakeFlexEncoder = upperIntakeSparkFlex.getEncoder();
        this.lowerIntakeFlexEncoder = lowerIntakeSparkFlex.getEncoder();
        this.setElevatorState = setElevatorState;
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
        if (intendedState != intakeState) {
            switch (intendedState) {
                case FULL -> {
                    intakeState = reachedVelocity && (upperMasterIntakeFlexEncoder.getVelocity() < Intake.Physical.INTAKE_VELOCIRTY_THREASHOLD / 55 || lowerIntakeFlexEncoder.getVelocity() < Intake.Physical.INTAKE_VELOCIRTY_THREASHOLD / 55) ? IntakeState.FULL : intakeState;

                    if (reachedVelocity && intakeState == IntakeState.FULL) {
                        reachedVelocity = false;
                        this.setElevatorState.accept(ElevatorState.DOWN);
                    }
                }
                case EMPTY -> {
                    intakeState = upperMasterIntakeFlexEncoder.getVelocity() - prevVelocityUp < Intake.Physical.DECELERATION_THRESHOLD || lowerIntakeFlexEncoder.getVelocity() - prevVelocityDown < Intake.Physical.DECELERATION_THRESHOLD ? IntakeState.EMPTY : intakeState;
                    reachedVelocity = false;
                }
            }
        }
        if (!reachedVelocity) {
            reachedVelocity = intendedState == IntakeState.FULL && (upperMasterIntakeFlexEncoder.getVelocity() > Intake.Physical.INTAKE_VELOCIRTY_THREASHOLD || lowerIntakeFlexEncoder.getVelocity() > Intake.Physical.INTAKE_VELOCIRTY_THREASHOLD);
        }
        prevVelocityUp = upperMasterIntakeFlexEncoder.getVelocity();
        prevVelocityDown = lowerIntakeFlexEncoder.getVelocity();
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
