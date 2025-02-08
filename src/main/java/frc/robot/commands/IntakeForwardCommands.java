package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Subsystems.Intake;
import frc.robot.subsystems.IntakeSubsystem;

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
