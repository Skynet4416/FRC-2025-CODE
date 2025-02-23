package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {

    TalonFX m_Kraken;

    public ClimbSubsystem(){
        this.m_Kraken = new TalonFX(99);
    }

    public void turnClimb(double power)
    {
        m_Kraken.set(power);;
    }
    
}
