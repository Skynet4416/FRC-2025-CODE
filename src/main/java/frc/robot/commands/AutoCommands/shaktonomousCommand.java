package frc.robot.commands.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;
public class shaktonomousCommand extends Command {
    private final CommandSwerveDrivetrain drive;
    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController rotationPID;
    private Pose2d robotpose;
    private Pose2d target = new Pose2d(5.0,5.0, Rotation2d.fromDegrees(30.0));
    private final double offset = 0.2;

    public shaktonomousCommand(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        xPID = new PIDController(0, 0, 0);
        yPID = new PIDController(0, 0, 0);
        rotationPID = new PIDController(8, 0, 0);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xPID.reset();
        yPID.reset();
        rotationPID.reset();
    }

    @Override
    public void execute() {
        boolean tv = LimelightHelpers.getTV("limelight");
        if (!tv) {
            return;
        }
        robotpose = LimelightHelpers.getBotPose2d("limelight");


        double xCorrection = xPID.calculate(robotpose.getX(), target.getX());
        double yCorrection = yPID.calculate(robotpose.getY(), target.getY());
        double rotationCorrection = rotationPID.calculate(robotpose.getRotation().getRadians(), target.getRotation().getRadians());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xCorrection, yCorrection, rotationCorrection,robotpose.getRotation());
        SwerveRequest request = drive.m_pathApplyRobotSpeeds.withSpeeds(chassisSpeeds);
        drive.setControl(request);
    }

    @Override
    public boolean isFinished() {
        double xError = Math.abs(robotpose.getX() - target.getX());
        double yError = Math.abs(robotpose.getY() - target.getY());
        return xError < offset && yError < offset;
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        SwerveRequest request = drive.m_pathApplyRobotSpeeds.withSpeeds(zeroSpeeds);
        drive.setControl(request);
    }
}
