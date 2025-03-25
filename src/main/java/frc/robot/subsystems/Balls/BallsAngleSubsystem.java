package frc.robot.subsystems.Balls;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
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
        this.angleSparkMax = new SparkMax(Constants.Subsystems.Balls.Motors.ANGLE_SPARK_MAX_CAN_ID,
                SparkLowLevel.MotorType.kBrushless);

        SparkMaxConfig angleConfig = new SparkMaxConfig();

        angleConfig.voltageCompensation(12).smartCurrentLimit(40).idleMode(SparkBaseConfig.IdleMode.kBrake)
                .inverted(true);
        angleSparkMax.configure(angleConfig, SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters);
        angleEncoder = new DutyCycleEncoder(Constants.Subsystems.Balls.Encoders.DUTY_CYCLE_ENCODER_PORT);
        this.anglePidController = new PIDController(Constants.Subsystems.Balls.PID.Angle.KP,
                Constants.Subsystems.Balls.PID.Angle.KI, Constants.Subsystems.Balls.PID.Angle.KD);
        anglePidController.enableContinuousInput(0, 1);
        this.anglePidController.setTolerance(0.005);
        SmartDashboard.putNumber("balls angle kp", Constants.Subsystems.Balls.PID.Angle.KP);
        SmartDashboard.putNumber("balls angle ki", Constants.Subsystems.Balls.PID.Angle.KI);
        SmartDashboard.putNumber("balls angle kd", Constants.Subsystems.Balls.PID.Angle.KD);
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

    public boolean atSetpoint() {
        return anglePidController.atSetpoint();
    }

    public void setVoltage(double voltage) {
        this.angleSparkMax.setVoltage(voltage);
    }

    public void setCalculated() {
        setVoltage(-MathUtil.clamp(anglePidController.calculate(this.getAngleFromZero()), -10, 10));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("balls relative angle", getAngleFromZero());
        SmartDashboard.putNumber("absolute angle", angleEncoder.get());
        SmartDashboard.putNumber("balls angle velocity", angleSparkMax.getEncoder().getVelocity());
        anglePidController.setPID(SmartDashboard.getNumber("balls angle kp", Constants.Subsystems.Balls.PID.Angle.KP),
                SmartDashboard.getNumber("balls angle ki", Constants.Subsystems.Balls.PID.Angle.KI),
                SmartDashboard.getNumber("balls angle kd", Constants.Subsystems.Balls.PID.Angle.KD));
    }
}
