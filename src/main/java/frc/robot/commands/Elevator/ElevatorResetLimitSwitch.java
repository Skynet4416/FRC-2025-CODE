package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorResetLimitSwitch extends Command {
    private final ElevatorSubsystem elevatorSubsystem;

    public ElevatorResetLimitSwitch(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        if (!elevatorSubsystem.elevatorDown()) {
            elevatorSubsystem.setPercentage(-Constants.Subsystems.Elevator.Controls.ELEVATOR_PERCENTAGE);
        }
    }
    
    // @Override
    // public boolean isFinished() {
    //     return elevatorSubsystem.elevatorDown();
    // }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPercentage(0);
    }
}
