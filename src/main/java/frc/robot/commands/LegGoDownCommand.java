package frc.robot.commands;

import frc.robot.Constants.Subsystems.DeepCage;
import frc.robot.subsystems.ClimbDeepSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class LegGoDownCommand extends Command {
    private final ClimbDeepSubsystem climbDeepSubsystem;

    public LegGoDownCommand(ClimbDeepSubsystem climbDeepSubsystem) {
        this.climbDeepSubsystem = climbDeepSubsystem;
        addRequirements(climbDeepSubsystem);
    }

    @Override
    public void initialize() {
        climbDeepSubsystem.moveMotor(DeepCage.Physical.percentage);
    }

    @Override
    public boolean isFinished() {
        return climbDeepSubsystem.limitSwitchChecker();
    }

    @Override
    public void end(boolean interrupted) {
        climbDeepSubsystem.stopMotor();
    }
}
