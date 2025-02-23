package frc.robot.commands.climb;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbClose extends Command {
    boolean dididoityesno = false;
    ClimbSubsystem climb;
    public ClimbClose(ClimbSubsystem climb) {
        this.climb = climb;
    }

    @Override
    public void initialize() {
    }
    public void execute() {
        climb.turnClimb(-0.5);
    }

    @Override
    public boolean isFinished() { 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
