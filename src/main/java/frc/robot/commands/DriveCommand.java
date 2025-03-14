package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveCommand extends Command {

    private final CommandSwerveDrivetrain driveSubsystem;
    private double wantedAngle;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier angleRadiansSupplier;
    private final BooleanSupplier manualOverride;

    public DriveCommand(CommandSwerveDrivetrain driveSubsystem,
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier,
            DoubleSupplier angleRadiansSupplier, BooleanSupplier manualOVerride) {
        this.driveSubsystem = driveSubsystem;

        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.angleRadiansSupplier = angleRadiansSupplier;
        this.manualOverride = manualOVerride;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        wantedAngle = Units.degreesToRadians(driveSubsystem.getGyroRotationInDegrees());
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("xSupplier", xSupplier.getAsDouble());
        SmartDashboard.putNumber("ySupplier", ySupplier.getAsDouble());
        SmartDashboard.putNumber("rSupplier", rotationSupplier.getAsDouble());
        if (manualOverride.getAsBoolean() || angleRadiansSupplier.getAsDouble() == -999) {
            wantedAngle += rotationSupplier.getAsDouble() * 0.02;
        } else {
            wantedAngle = angleRadiansSupplier.getAsDouble();
        }
        SmartDashboard.putNumber("wanted angel", wantedAngle);

        driveSubsystem.setControl(
                new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                        .withVelocityX(xSupplier.getAsDouble()).withVelocityY(ySupplier.getAsDouble())
                        .withRotationalRate(MathUtil.clamp(driveSubsystem.calculateRotation(wantedAngle), -6.283 * 1.5,
                                6.283 * 1.5)));
    }
}
