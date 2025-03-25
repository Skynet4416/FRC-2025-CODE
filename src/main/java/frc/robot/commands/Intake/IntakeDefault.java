package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeDefault extends Command {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeDefault(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setDistance(intakeSubsystem.getDistance());
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.moveMotor(0);
    }

}
