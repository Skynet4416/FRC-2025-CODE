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
    private double wantedAngle;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;

    public DriveCommand(CommandSwerveDrivetrain driveSubsystem,
                        DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        this.driveSubsystem = driveSubsystem;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(driveSubsystem);
    }


    @Override
public void initialize() {
        wantedAngle = driveSubsystem.getState().Pose.getRotation().getDegrees();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("xSupplier", xSupplier.getAsDouble());
        SmartDashboard.putNumber("ySupplier", ySupplier.getAsDouble());
        SmartDashboard.putNumber("rSupplier", rotationSupplier.getAsDouble());
        driveSubsystem.setControl(
                new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withVelocityX(xSupplier.getAsDouble()).withVelocityY(ySupplier.getAsDouble())
                        .withRotationalRate(rotationSupplier.getAsDouble()));
    }
}
