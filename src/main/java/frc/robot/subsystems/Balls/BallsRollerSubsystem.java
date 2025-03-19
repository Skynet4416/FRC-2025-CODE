package frc.robot.subsystems.Balls;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallsRollerSubsystem extends SubsystemBase {
    private final SparkMax rollerSparkMax;
    private final SparkClosedLoopController rollerClosedLoopController;
    private final RelativeEncoder rollerEncoder;

    public BallsRollerSubsystem() {
        this.rollerSparkMax = new SparkMax(Constants.Subsystems.Balls.Motors.ROLLER_SPARK_MAX_CAN_ID,
                SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.voltageCompensation(12).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake).closedLoop
                .pid(Constants.Subsystems.Balls.PID.Roller.KP, Constants.Subsystems.Balls.PID.Roller.KI,
                        Constants.Subsystems.Balls.PID.Roller.KD);
        rollerSparkMax.configure(rollerConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        rollerClosedLoopController = this.rollerSparkMax.getClosedLoopController();
        this.rollerEncoder = rollerSparkMax.getEncoder();

    }

    public void setRollerPercentage(double percentage) {
        this.rollerSparkMax.set(percentage);
    }

    public void setRollerDistance(double distance) {
        rollerClosedLoopController.setReference(distance, SparkBase.ControlType.kPosition);
    }

    public double getRollerDistance() {
        return this.rollerEncoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("balls roller distance", getRollerDistance());
        SmartDashboard.putNumber("balls roller velocity", rollerEncoder.getVelocity());

    }
}
