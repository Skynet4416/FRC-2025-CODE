package frc.robot.commands.Intake;

import frc.robot.Constants.Subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
public class IntakeOutwardCommands extends Command{
    private final IntakeSubsystem intakeSubsystem;

    public IntakeOutwardCommands(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeSubsystem.moveMotor(-Intake.Physical.INTAKE_PERCENTAGE);
    }
    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stopMotor();
    }
}
