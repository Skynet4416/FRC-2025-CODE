package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsystems.DeepCage;

public class ClimbDeepSubsystem extends SubsystemBase {

    private SparkMax DeepClimbSparkMax;
    private DigitalInput ClimbLegLimitSwitch = new DigitalInput(0);


    public ClimbDeepSubsystem() {
        DeepClimbSparkMax = new SparkMax(DeepCage.Motors.DEEP_CAGE_MAX_MOTOR_ID, MotorType.kBrushless);
    }
    
    public boolean limitSwitchChecker(){
        return this.ClimbLegLimitSwitch.get();
    }

    public void moveMotor(double percentage) {
        this.DeepClimbSparkMax.set(percentage); 
    }

    public void stopMotor() {
        this.DeepClimbSparkMax.set(0);
    }

}
