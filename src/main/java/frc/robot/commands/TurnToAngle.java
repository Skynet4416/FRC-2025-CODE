package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class TurnToAngle extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private double wantedAngleInRadians;

    public TurnToAngle(CommandSwerveDrivetrain drivetrain, double wantedAngleInRadians) {
        this.drivetrain = drivetrain;
        this.wantedAngleInRadians = wantedAngleInRadians;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("setpoint", Units.degreesToRadians(wantedAngleInRadians));
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("velocity", drivetrain.calculateRotation(wantedAngleInRadians));
        drivetrain.setControl(
                new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withRotationalRate(MathUtil.clamp(drivetrain.calculateRotation(wantedAngleInRadians), -6.283 * 1.5, 6.283 * 1.5)
                ));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(drivetrain.getGyroRotationInDegrees() - Units.radiansToDegrees(wantedAngleInRadians)) < 1;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
                new SwerveRequest.FieldCentric().withDriveRequestType(SwerveModule.DriveRequestType.Velocity).withRotationalRate(0));

    }

}
