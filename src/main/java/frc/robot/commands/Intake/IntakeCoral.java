package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeCoral extends Command {
    private IntakeSubsystem intakeSubsystem;
    private boolean reachedVelocity = false;

    public IntakeCoral(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.moveMotor(Constants.States.Intake.INTAKE_PERCEHNTAGE);
    }

    @Override
    public void execute() {
        if (!reachedVelocity && intakeSubsystem.getIntakeVelocity() > Constants.States.Intake.INTAKE_THREASHOLD) {
            reachedVelocity = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.moveMotor(0);
        if (!interrupted) {
            intakeSubsystem.setState(IntakeState.FULL);
        }
    }

    @Override
    public boolean isFinished() {
        return reachedVelocity && intakeSubsystem.getIntakeVelocity() < Constants.States.Intake.INTAKE_THREASHOLD;
    }
}
