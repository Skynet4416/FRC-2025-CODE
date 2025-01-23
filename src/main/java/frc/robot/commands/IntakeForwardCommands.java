package frc.robot.commands;

import frc.robot.Constants.Subsystems.Intake;
import frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
public class IntakeForwardCommands extends Command{
    private final IntakeSubsystem intakeSubsystem;

    public IntakeForwardCommands(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.moveMotor(Intake.Physical.INTAKE_PERCENTAGE);
    }
    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stopMotor();
    }
}
