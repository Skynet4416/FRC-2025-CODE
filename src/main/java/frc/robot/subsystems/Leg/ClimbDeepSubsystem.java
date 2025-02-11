package frc.robot.subsystems.Leg;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.DeepCage;

public class ClimbDeepSubsystem extends SubsystemBase {

    private final SparkMax DeepClimbSparkMax;
    private final DigitalInput climbLegLimitSwitch;


    public ClimbDeepSubsystem() {
        DeepClimbSparkMax = new SparkMax(DeepCage.Motors.DEEP_CAGE_MAX_MOTOR_ID, MotorType.kBrushless);
        climbLegLimitSwitch = new DigitalInput(DeepCage.Sensors.LEG_LIMIT_SWITCH_CHANNEL);
    }

    public boolean limitSwitchChecker() {
        return this.climbLegLimitSwitch.get();
    }

    public void moveMotor(double percentage) {
        this.DeepClimbSparkMax.set(percentage);
    }

    public void stopMotor() {
        this.DeepClimbSparkMax.set(0);
    }

}
