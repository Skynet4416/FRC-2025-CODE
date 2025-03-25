package frc.robot.commands.Balls;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Balls.BallsAngleSubsystem;

public class BallsAngleToAngle extends Command {
    private final BallsAngleSubsystem ballsAngleSubsystem;
    private final double angle;

    public BallsAngleToAngle(BallsAngleSubsystem ballsAngleSubsystem, double angle) {
        addRequirements(ballsAngleSubsystem);
        this.ballsAngleSubsystem = ballsAngleSubsystem;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        ballsAngleSubsystem.setAngle(angle);
    }

    @Override
    public void execute() {
        ballsAngleSubsystem.setCalculated();
    }

    // @Override
    // public boolean isFinished() {
    //     return ballsAngleSubsystem.atSetpoint();
    // }

    @Override
    public void end(boolean interrupted) {
        ballsAngleSubsystem.setAnglePercentage(0);
    }

}
