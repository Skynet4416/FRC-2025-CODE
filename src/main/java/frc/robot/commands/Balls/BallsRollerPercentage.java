package frc.robot.commands.Balls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Balls.BallsAngleSubsystem;
import frc.robot.subsystems.Balls.BallsRollerSubsystem;

public class BallsRollerPercentage extends Command {
    private final BallsRollerSubsystem ballsRollerSubsystem;
    private final double percentage;

    public BallsRollerPercentage(BallsRollerSubsystem ballsRollerSubsystem, double percentage) {
        this.ballsRollerSubsystem = ballsRollerSubsystem;
        this.percentage = percentage;
        addRequirements(ballsRollerSubsystem);
    }

    @Override
    public void initialize() {
        ballsRollerSubsystem.setRollerPercentage(percentage);
    }

    @Override
    public void end(boolean interrupted) {
        ballsRollerSubsystem.setRollerPercentage(0);
    }
}
