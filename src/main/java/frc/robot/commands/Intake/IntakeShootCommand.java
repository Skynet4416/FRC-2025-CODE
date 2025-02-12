package frc.robot.commands.Intake;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.meth.Distance;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import java.util.function.Supplier;

public class IntakeShootCommand extends Command {
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<RobotState> stateSupplier;
    private final Supplier<Pose2d> positionalSupplier;

    public IntakeShootCommand(IntakeSubsystem intakeSubsystem, Supplier<RobotState> stateSupplier, Supplier<Pose2d> positionSupplier) {
        this.intakeSubsystem = intakeSubsystem;
        this.stateSupplier = stateSupplier;
        this.positionalSupplier = positionSupplier;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        if (stateSupplier.get() == RobotState.SCORE) {
            if (Distance.isPointNearLinesSegment(positionalSupplier.get().getTranslation(),
                    FieldConstants.Reef.centerFaces, FieldConstants.Reef.faceLength, Constants.States.Intake.RADIUS_IN_METERS) != null)
                intakeSubsystem.moveMotor(Constants.Subsystems.Intake.Physical.INTAKE_PERCENTAGE);
        }
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getState() == IntakeState.EMPTY;
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopMotor();
    }
}
