package frc.robot.commands.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import java.util.function.Supplier;

import static frc.robot.meth.Distance.isPointNearLineSegment;

public class IntakeBasedOnStateCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private Supplier<RobotState> stateSupplier;
    private Supplier<Pose2d> positionalSupplier;

    public IntakeBasedOnStateCommand(IntakeSubsystem intakeSubsystem, Supplier<RobotState> stateSupplier, Supplier<Pose2d> positionSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.stateSupplier = stateSupplier;
        this.positionalSupplier = positionSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        if (stateSupplier.get() == RobotState.INTAKE && intakeSubsystem.getState() == IntakeState.EMPTY) {
            if (isPointNearLineSegment(positionalSupplier.get().getTranslation(), FieldConstants.CoralStation.leftCenterFace, FieldConstants.CoralStation.stationLength, Constants.States.Intake.radiusInMeters) || isPointNearLineSegment(positionalSupplier.get().getTranslation(), FieldConstants.CoralStation.rightCenterFace, FieldConstants.CoralStation.stationLength, Constants.States.Intake.radiusInMeters))
                intakeSubsystem.moveMotor(Constants.Subsystems.Intake.Physical.INTAKE_PERCENTAGE);
        } else {
            intakeSubsystem.stopMotor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotor();
    }
}
