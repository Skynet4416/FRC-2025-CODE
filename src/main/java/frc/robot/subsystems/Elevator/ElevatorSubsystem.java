package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motorLeader;
    private final SparkMax motorRightSlave;
    private final SparkMax motorLeftSlave1;
    private final SparkMax motorleftSlave2;

    private final RelativeEncoder masterEncoder;
    private final SparkClosedLoopController masterClosedLoopController;
    private ElevatorState state;
    private final DigitalInput hallEffect;

    private double setpoint = 0;
    private SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private boolean resetedEncoder = false;

    public ElevatorSubsystem() {
        hallEffect = new DigitalInput(Elevator.Sensors.HALL_EFFECT_PORT);
        motorLeader = new SparkMax(Elevator.Motors.MASTER_CAN_ID, MotorType.kBrushless);
        motorRightSlave = new SparkMax(Elevator.Motors.RIGHT_SLAVE_CAN_ID, MotorType.kBrushless);
        motorLeftSlave1 = new SparkMax(Elevator.Motors.LEFT_SLAVE_1_CAN_ID, MotorType.kBrushless);
        motorleftSlave2 = new SparkMax(Elevator.Motors.LEFT_SLAVE_2_CAN_ID, MotorType.kBrushless);
        state = ElevatorState.DOWN;

        leaderConfig.inverted(true);
        leaderConfig.encoder.positionConversionFactor(Elevator.Physical.POSITION_CONVERSION_FACTOR);
        leaderConfig.encoder.velocityConversionFactor(Elevator.Physical.VELOCITY_CONVERSION_FACTOR);
        leaderConfig.smartCurrentLimit(25).voltageCompensation(12).idleMode(SparkBaseConfig.IdleMode.kBrake);
        leaderConfig.closedLoop.pid(Elevator.PID.KP, Elevator.PID.KI, Elevator.PID.KD).maxMotion.maxVelocity(Elevator.Physical.MAX_VELOCITY_IN_MPS).maxAcceleration(Elevator.Physical.MAX_ACCELERATION_IN_MPS_SQUARED).allowedClosedLoopError(Elevator.Controls.HEIGHT_THRESHOLD_IN_METERS);

        motorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig rightSlaveConfig = new SparkMaxConfig();
        rightSlaveConfig.smartCurrentLimit(25).voltageCompensation(12).idleMode(SparkBaseConfig.IdleMode.kBrake);
        rightSlaveConfig.follow(motorLeader);

        SparkMaxConfig leftSlaveConfig = new SparkMaxConfig();

        leftSlaveConfig.follow(motorLeader, true).smartCurrentLimit(25).idleMode(SparkBaseConfig.IdleMode.kBrake);
        leftSlaveConfig.inverted(true);
        motorRightSlave.configure(rightSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorLeftSlave1.configure(leftSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorleftSlave2.configure(leftSlaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.masterEncoder = this.motorLeader.getEncoder();
        this.masterClosedLoopController = this.motorLeader.getClosedLoopController();
    }


    public void setPercentage(double percentage) {
        motorLeader.set(percentage);
    }

    public double getElevatorDistanceInMeter() {
        return this.masterEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current elevator position meters", masterEncoder.getPosition());
        SmartDashboard.putNumber("Current elevator Velocity mps", masterEncoder.getVelocity());
        SmartDashboard.putBoolean("elevator limit switch", this.elevatorDown());

        if (elevatorDown() && masterEncoder.getVelocity() <= 0) {
            masterEncoder.setPosition(0);
            resetedEncoder = true;
        }
        SmartDashboard.putNumber("elevator voltage", motorLeader.getOutputCurrent());
        SmartDashboard.putNumber("elevator setpoint", setpoint);

        if (Math.abs(getElevatorDistanceInMeter() - Elevator.Physical.MAX_HEIGHT_IN_METERS) < Elevator.Controls.HEIGHT_THRESHOLD_IN_METERS && motorLeader.get() > 0 || (resetedEncoder && getElevatorDistanceInMeter() < 0)) {
            setPercentage(0);
        }

    }

    public void setIntendedState(ElevatorState state) {
        this.state = state;
    }

    public ElevatorState getIntendedState() {
        return this.state;
    }

    public boolean elevatorDown() {
        return !hallEffect.get();
    }

    public boolean elevatorAtSetpoint(double setpoint) {
        return Math.abs(getElevatorDistanceInMeter() - setpoint) < Elevator.Controls.HEIGHT_THRESHOLD_IN_METERS;
    }

}
