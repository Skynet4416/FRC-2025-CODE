package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.Elevator;

public class ElevatorSubsystem extends SubsystemBase {

    private final SparkMax motorLeader;
    private final SparkMax motorSlave1;
    private final SparkMax motorSlave2;
    private final SparkMax motorSlave3;

    private final RelativeEncoder masterEncoder;
    private final SparkClosedLoopController masterClosedLoopController;

    public ElevatorSubsystem() {
        motorLeader = new SparkMax(Elevator.Motors.MASTER_CAN_ID, MotorType.kBrushless);
        motorSlave1 = new SparkMax(Elevator.Motors.SLAVE_1_CAN_ID, MotorType.kBrushless);
        motorSlave2 = new SparkMax(Elevator.Motors.SLAVE_2_CAN_ID, MotorType.kBrushless);
        motorSlave3 = new SparkMax(Elevator.Motors.SLAVE_3_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        leaderConfig.encoder.positionConversionFactor(Elevator.Physical.WHEEL_RADIUS_IN_METERS * 2 * Math.PI / Elevator.Physical.GEAR_RATIO);
        leaderConfig.encoder.velocityConversionFactor(Elevator.Physical.WHEEL_RADIUS_IN_METERS * 2 * Math.PI / (Elevator.Physical.GEAR_RATIO * 60));
        // move math to constants

        leaderConfig.closedLoop.pid(Elevator.PID.KP, Elevator.PID.KI, Elevator.PID.KD).maxMotion.maxVelocity(Elevator.Physical.MAX_VELOCITY_IN_MPS).maxAcceleration(Elevator.Physical.MAX_ACCELERATION_IN_MPS_SQUARED);

        motorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig slaveConfig = new SparkMaxConfig();

        slaveConfig.follow(motorLeader);

        motorSlave1.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSlave2.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSlave3.configure(slaveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.masterEncoder = this.motorLeader.getEncoder();
        this.masterClosedLoopController = this.motorLeader.getClosedLoopController();

    }

    public void setElevatorDistanceInMeters(double targetDistanceInMeters) {
        double clampedDistanceInMeters = MathUtil.clamp(targetDistanceInMeters, 0, Elevator.Physical.MAX_HEIGHT_IN_METERS);

        masterClosedLoopController.setReference(clampedDistanceInMeters,
                ControlType.kMAXMotionPositionControl);
    }

    public void setPercentage(double percentage) {
        motorLeader.set(percentage);
    }

    public double getElevatorDistanceInMeter() {
        return this.masterEncoder.getPosition();
    }

    public void StopMotors() {
        motorLeader.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current elevator position meters", masterEncoder.getPosition());
        SmartDashboard.putNumber("Current elevator Velocity mps", masterEncoder.getVelocity());
    }

}
