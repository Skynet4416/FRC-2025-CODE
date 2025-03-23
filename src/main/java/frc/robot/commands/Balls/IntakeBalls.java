package frc.robot.commands.Balls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Balls.BallsRollerSubsystem;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeBalls extends Command {
    private BallsRollerSubsystem ballsRollerSubsystem;
    private boolean reachedVelocity;

    public IntakeBalls(BallsRollerSubsystem ballsRollerSubsystem) {
        this.ballsRollerSubsystem = ballsRollerSubsystem;
        addRequirements(ballsRollerSubsystem);
    }

    @Override
    public void initialize() {
        ballsRollerSubsystem.setRollerPercentage(1);
        this.reachedVelocity = false;
    }

    @Override
    public void execute() {
        if (!reachedVelocity && ballsRollerSubsystem.getRollerVelocity() > Constants.States.Balls.ROLLER_THREASHOLD) {
            reachedVelocity = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        ballsRollerSubsystem.setRollerPercentage(0);
        if (!interrupted) {
            ballsRollerSubsystem.setBallsIn(true);
        }
        System.out.println("Balls END");
    }

    @Override
    public boolean isFinished() {
        return reachedVelocity && ballsRollerSubsystem.getRollerVelocity() < Constants.States.Balls.ROLLER_THREASHOLD;
    }
}
