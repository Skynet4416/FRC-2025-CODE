package frc.robot.subsystems.Balls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class BallsKeepAtAngle extends Command {
    private final BallsAngleSubsystem ballsAngleSubsystem;

    public BallsKeepAtAngle(BallsAngleSubsystem ballsAngleSubsystem) {
        addRequirements(ballsAngleSubsystem);
        this.ballsAngleSubsystem = ballsAngleSubsystem;
    }

    @Override
    public void initialize() {
        ballsAngleSubsystem.setAnglePercentage(Constants.States.None.BALLS_RESTING_ANGLE);
    }

    @Override
    public void execute() {
        ballsAngleSubsystem.setCalculated();
    }

    @Override
    public void end(boolean interrupted) {
        ballsAngleSubsystem.setAnglePercentage(0);
    }
}
