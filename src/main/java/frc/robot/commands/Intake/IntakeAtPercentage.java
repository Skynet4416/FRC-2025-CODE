package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeAtPercentage extends Command{
    private IntakeSubsystem intakeSubsystem;
    private double percentage;

    public IntakeAtPercentage(IntakeSubsystem intakeSubsystem, double percentage){
        this.intakeSubsystem = intakeSubsystem;
        this.percentage = percentage;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.moveMotor(percentage);
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.moveMotor(0);
    }
}