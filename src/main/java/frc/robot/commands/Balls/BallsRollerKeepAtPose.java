package frc.robot.commands.Balls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Balls.BallsRollerSubsystem;

public class BallsRollerKeepAtPose extends Command {
    private final BallsRollerSubsystem ballsRollerSubsystem;

    public BallsRollerKeepAtPose(BallsRollerSubsystem ballsRollerSubsystem) {
        this.ballsRollerSubsystem = ballsRollerSubsystem;

        addRequirements(ballsRollerSubsystem);
    }

    @Override
    public void initialize() {
        ballsRollerSubsystem.setRollerDistance(ballsRollerSubsystem.getRollerDistance());
    }

    @Override
    public void end(boolean interrupted) {
        ballsRollerSubsystem.setRollerPercentage(0);
    }
}
