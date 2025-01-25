package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier speedModeSupplier;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, DoubleSupplier speedModeSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.speedModeSupplier = speedModeSupplier;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = xSupplier.getAsDouble() / (speedModeSupplier.getAsDouble() > 0.5 ? 8.0 : 1.0);
        double ySpeed = ySupplier.getAsDouble() / (speedModeSupplier.getAsDouble() > 0.5 ? 8.0 : 1.0);
        double rotSpeed = rotationSupplier.getAsDouble() / (speedModeSupplier.getAsDouble() > 0.5 ? 8.0 : 1.0);

        driveSubsystem.getDrivetrain().setControl(
            driveSubsystem.getDriveRequest()
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.getDrivetrain().setControl(
            driveSubsystem.getDriveRequest()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
