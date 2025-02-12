package frc.robot.commands.Elevator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.meth.Distance;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

import java.util.function.Supplier;

import static frc.robot.meth.Distance.isPointNearLineSegment;
import static frc.robot.meth.Distance.isPointNearLinesSegment;

public class ElevatorBasedOnStateCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<RobotState> robotStateSupplier;
    private final Supplier<Pose2d> positionSupplier;
    private RobotState intendedState;

    public ElevatorBasedOnStateCommand(ElevatorSubsystem elevatorSubsystem, Supplier<RobotState> robotStateSupplier, Supplier<Pose2d> positionSupplier) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.robotStateSupplier = robotStateSupplier;
        this.positionSupplier = positionSupplier;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        switch (robotStateSupplier.get()) {
            case INTAKE:
                if (isPointNearLinesSegment(positionSupplier.get().getTranslation(),
                        new Pose2d[]{FieldConstants.CoralStation.leftCenterFace, FieldConstants.CoralStation.rightCenterFace},
                        FieldConstants.CoralStation.stationLength, Constants.States.Intake.RADIUS_IN_METERS) != null) {
                    elevatorSubsystem.setElevatorDistanceInMeters(Constants.States.Score.ELEVATOR_HEIGHT);
                    break;
                }
            case SCORE:
                if ((Distance.isPointNearLinesSegment(positionSupplier.get().getTranslation(),
                        FieldConstants.Reef.centerFaces, FieldConstants.Reef.faceLength, Constants.States.Intake.RADIUS_IN_METERS) != null))
                    elevatorSubsystem.setElevatorDistanceInMeters(Constants.States.Score.ELEVATOR_HEIGHT);
                break;
            case CLIMB:
                if (elevatorSubsystem.getState() == ElevatorState.DOWN) {
                    elevatorSubsystem.setElevatorDistanceInMeters(Constants.States.Climb.ELEVATOR_HEIGHT);
                    elevatorSubsystem.setState(ElevatorState.UP);
                }
                break;
            default:
                elevatorSubsystem.setElevatorDistanceInMeters(Constants.States.None.ELEVATOR_HEIGHT);
                break;
        }
    }
}
