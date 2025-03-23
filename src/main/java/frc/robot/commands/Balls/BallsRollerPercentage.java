package frc.robot.commands.Balls;

import java.util.function.Consumer;
import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Balls.BallsAngleSubsystem;
import frc.robot.subsystems.Balls.BallsRollerSubsystem;

public class BallsRollerPercentage extends Command {
    private final BallsRollerSubsystem ballsRollerSubsystem;
    private final double percentage;
    private final Consumer<Boolean> endFunction;

    public BallsRollerPercentage(BallsRollerSubsystem ballsRollerSubsystem, double percentage,
            Consumer<Boolean> endFunction) {
        this.ballsRollerSubsystem = ballsRollerSubsystem;
        this.percentage = percentage;
        this.endFunction = endFunction;
        addRequirements(ballsRollerSubsystem);
    }

    @Override
    public void initialize() {
        ballsRollerSubsystem.setRollerPercentage(percentage);
    }

    @Override
    public void end(boolean interrupted) {
        ballsRollerSubsystem.setRollerPercentage(0);
        endFunction.accept(interrupted);
    }
}
