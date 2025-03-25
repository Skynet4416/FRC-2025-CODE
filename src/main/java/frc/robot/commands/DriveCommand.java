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
        if (manualOverride.getAsBoolean() || angleRadiansSupplier.getAsDouble() == -999) {
            wantedAngle = this.driveSubsystem.getState().Pose.getRotation().getRadians();
        } else {
            wantedAngle = angleRadiansSupplier.getAsDouble();
        }
    }

    public boolean closeToZero(double value, double threashold) {
        return Math.abs(value) < threashold;
    }

    public boolean closeToZero(double value) {
        return closeToZero(value, 0.1);
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

        boolean xSuppliersLock = closeToZero(xSupplier.getAsDouble());
        SmartDashboard.putBoolean("x supplier lock", xSuppliersLock);
        boolean ySuppliersLock = closeToZero(ySupplier.getAsDouble());
        SmartDashboard.putBoolean("y supplier lock", ySuppliersLock);
        boolean rotationSupplierLock = closeToZero(rotationSupplier.getAsDouble());
        SmartDashboard.putBoolean("r supplier lock", rotationSupplierLock);

        boolean vxLock = closeToZero(driveSubsystem.getState().Speeds.vxMetersPerSecond);
        SmartDashboard.putBoolean("vx lock", vxLock);
        boolean vyLock = closeToZero(driveSubsystem.getState().Speeds.vyMetersPerSecond);
        SmartDashboard.putBoolean("vy lock", vyLock);
        boolean vrLock = closeToZero(driveSubsystem.getState().Speeds.omegaRadiansPerSecond,0.5);
        SmartDashboard.putBoolean("vr lock", vrLock);

        boolean x = xSuppliersLock && ySuppliersLock && rotationSupplierLock && vxLock && vyLock && vrLock;
        SmartDashboard.putBoolean("swerve lock", x);
        SmartDashboard.putNumber("vr value", Math.abs(driveSubsystem.getState().Speeds.omegaRadiansPerSecond - 0.5));
        if (x) {
            driveSubsystem.setControl(new SwerveRequest.SwerveDriveBrake());
        } else {
            driveSubsystem.setControl(
                    new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                            .withVelocityX(xSupplier.getAsDouble()).withVelocityY(ySupplier.getAsDouble())
                            .withRotationalRate(
                                    MathUtil.clamp(driveSubsystem.calculateRotation(wantedAngle), -6.283 * 1.5,
                                            6.283 * 1.5)));
        }
    }
}
