package frc.robot.subsystems.Balls;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallsAngleSubsystem extends SubsystemBase {
    private final SparkMax angleSparkMax;


    private final DutyCycleEncoder angleEncoder;
    private final PIDController anglePidController;

    public BallsAngleSubsystem() {
        this.angleSparkMax = new SparkMax(Constants.Subsystems.Balls.Motors.ANGLE_SPARK_MAX_CAN_ID, SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        SparkMaxConfig angleConfig = new SparkMaxConfig();

        rollerConfig.voltageCompensation(12).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake).closedLoop.pid(Constants.Subsystems.Balls.PID.Roller.KP, Constants.Subsystems.Balls.PID.Roller.KI, Constants.Subsystems.Balls.PID.Roller.KD);
        angleConfig.voltageCompensation(12).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake);

        angleEncoder = new DutyCycleEncoder(Constants.Subsystems.Balls.Encoders.DUTY_CYCLE_ENCODER_PORT);
        this.anglePidController = new PIDController(Constants.Subsystems.Balls.PID.Angle.KP, Constants.Subsystems.Balls.PID.Angle.KI, Constants.Subsystems.Balls.PID.Angle.KD);
        this.anglePidController.setTolerance(1);
    }

    public double getAngleFromZero() {
        return angleEncoder.get() - Constants.Subsystems.Balls.Physical.Angle.ZERO_ABSOLUTE_ANGLE;
    }


    public void setAnglePercentage(double percentage) {
        this.angleSparkMax.set(percentage);
    }

    public void setAngle(double angle) {
        anglePidController.setSetpoint(angle);
    }

    public void setVoltage(double voltage) {
        this.angleSparkMax.setVoltage(voltage);
    }

    public void setCalculated() {
        setVoltage(anglePidController.calculate(this.getAngleFromZero()));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("balls relative angle", getAngleFromZero());
        SmartDashboard.putNumber("absolute angle", angleEncoder.get());
    }
}
