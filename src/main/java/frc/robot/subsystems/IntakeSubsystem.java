package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.Intake;
public class IntakeSubsystem extends SubsystemBase {

    private SparkFlex intakeSparkFlex;

    private RelativeEncoder intakeFlexEncoder;

    public IntakeSubsystem() {
        intakeSparkFlex = new SparkFlex(Intake.Motors.SPARK_FLEX_ID, MotorType.kBrushless);
        this.intakeFlexEncoder = intakeSparkFlex.getEncoder();
    }

    public void moveMotor(double percentage) {
        this.intakeSparkFlex.set(percentage);
    }

    public void stopMotor() {
        this.intakeSparkFlex.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("current percentage", intakeSparkFlex.get());
        SmartDashboard.putNumber("current velcoity RPM", intakeFlexEncoder.getVelocity());
    }

}
