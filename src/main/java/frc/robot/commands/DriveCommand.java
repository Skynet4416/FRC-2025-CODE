package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.meth.Distance.isPointNearLinesSegment;

public class DriveCommand extends Command {
    private final CommandSwerveDrivetrain driveSubsystem;
    private final Supplier<RobotState> stateSupplier;
    private double wantedAngle;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier manualOverride;

    public DriveCommand(CommandSwerveDrivetrain driveSubsystem, Supplier<RobotState> stateSupplier,
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier,
            BooleanSupplier manualOverride) {
        this.driveSubsystem = driveSubsystem;
        this.stateSupplier = stateSupplier;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.manualOverride = manualOverride;
        addRequirements(driveSubsystem);
    }

    public Pose2d getPosition() {
        return driveSubsystem.getState().Pose;
    }

    @Override
    public void initialize() {
        wantedAngle = driveSubsystem.getState().Pose.getRotation().getDegrees();
    }

    @Override
    public void execute() {
        switch (stateSupplier.get()) {
            case SCORE:
                Pose2d scorePoint = isPointNearLinesSegment(getPosition().getTranslation(),
                        FieldConstants.Reef.centerFaces, FieldConstants.Reef.faceLength,
                        Constants.States.Score.RADIUS_IN_METERS);
                if (scorePoint != null && !manualOverride.getAsBoolean()) {
                    wantedAngle = scorePoint.getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees();
                    break;
                }
            case INTAKE:
                Pose2d intakePoint = isPointNearLinesSegment(getPosition().getTranslation(),
                        new Pose2d[] { FieldConstants.CoralStation.leftCenterFace,
                                FieldConstants.CoralStation.rightCenterFace },
                        FieldConstants.CoralStation.stationLength, Constants.States.Intake.RADIUS_IN_METERS);
                if (intakePoint != null && !manualOverride.getAsBoolean()) {
                    wantedAngle = intakePoint.getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees();
                    break;
                }
            default:
                wantedAngle += rotationSupplier.getAsDouble() * Units.millisecondsToSeconds(20);
        }

        SmartDashboard.putNumber("drive x", xSupplier.getAsDouble());
        SmartDashboard.putNumber("drive y", ySupplier.getAsDouble());
        SmartDashboard.putNumber("drive theta", wantedAngle);
        driveSubsystem.setControl(
                new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withVelocityX(xSupplier.getAsDouble()).withVelocityY(ySupplier.getAsDouble())
                        .withRotationalRate(rotationSupplier.getAsDouble()));
    }
}
