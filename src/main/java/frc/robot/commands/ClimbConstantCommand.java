package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb.ClimbSubsystem;

public class ClimbConstantCommand extends Command {
    private ClimbSubsystem climbSubsystem;

    public ClimbConstantCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        addRequirements(climbSubsystem);
    }

    @Override
    public void execute() {
        climbSubsystem.setPercentage(-1);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.setPercentage(0);
    }
}
