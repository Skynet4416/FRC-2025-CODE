package frc.robot.subsystems.Climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
    private final SparkMax climbLeader;
    private final SparkMax climbSlave;
    private final RelativeEncoder masterEncoder;
    private final SparkClosedLoopController masterClosedLoopController;
    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();

    public ClimbSubsystem() {
        climbLeader = new SparkMax(Constants.Subsystems.Climb.Motors.MASTER_CAN_ID, SparkLowLevel.MotorType.kBrushless);
        climbSlave = new SparkMax(Constants.Subsystems.Climb.Motors.SLAVE_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        leaderConfig.encoder.positionConversionFactor(Constants.Subsystems.Climb.Physical.POSITION_CONVERSION_FACTOR);
        leaderConfig.encoder.velocityConversionFactor(Constants.Subsystems.Climb.Physical.VELOCITY_CONVERSION_FACTOR);
        leaderConfig.smartCurrentLimit(80).voltageCompensation(12).idleMode(SparkBaseConfig.IdleMode.kBrake);
        leaderConfig.closedLoop.pid(Constants.Subsystems.Climb.PID.KP, Constants.Subsystems.Climb.PID.KI, Constants.Subsystems.Climb.PID.KD);

        climbLeader.configure(leaderConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        SparkMaxConfig slaveConfig = new SparkMaxConfig();
        slaveConfig.smartCurrentLimit(80).voltageCompensation(12).idleMode(SparkBaseConfig.IdleMode.kBrake);
        slaveConfig.follow(climbLeader);

        climbSlave.configure(slaveConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

        masterEncoder = climbLeader.getEncoder();
        masterClosedLoopController = climbLeader.getClosedLoopController();
    }

    public void setPercentage(double percentage) {
        this.climbLeader.set(percentage);
    }

    public void setDistance(double distanceInMeters) {
        this.masterClosedLoopController.setReference(distanceInMeters, SparkBase.ControlType.kPosition);
    }

    public double getClimbDistanceInMeters() {
        return this.masterEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climb distance", getClimbDistanceInMeters());
        SmartDashboard.putNumber("climb velocity", this.masterEncoder.getVelocity());
    }
}
