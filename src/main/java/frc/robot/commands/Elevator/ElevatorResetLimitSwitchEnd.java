package frc.robot.commands.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorResetLimitSwitchEnd extends Command{
     private final ElevatorSubsystem elevatorSubsystem;

    public ElevatorResetLimitSwitchEnd(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        if (!elevatorSubsystem.elevatorDown()) {
            elevatorSubsystem.setPercentage(MathUtil.clamp((elevatorSubsystem.getElevatorDistanceInMeter() / 0.3) * -Constants.Subsystems.Elevator.Controls.ELEVATOR_PERCENTAGE*4, -Constants.Subsystems.Elevator.Controls.ELEVATOR_PERCENTAGE, -0.1));
        }
        else{
            elevatorSubsystem.setPercentage(0);
        }
    }
    
    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorDown();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPercentage(0);
    }
}
