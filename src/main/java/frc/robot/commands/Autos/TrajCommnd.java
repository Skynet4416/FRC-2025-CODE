package frc.robot.commands.Autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class TrajCommnd extends Command {

    private Command followCommand;
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final double[] arr;
    private final CommandSwerveDrivetrain driveSubsystem;

    public TrajCommnd(AutoFactory autoFactory, String pathName, CommandSwerveDrivetrain drivetrain) {
        followCommand = autoFactory.trajectoryCmd(pathName);
        Pose2d[] poses = Choreo.loadTrajectory(pathName).get().getPoses();
        arr = new double[poses.length * 3];
        int ndx = 0;
        for (Pose2d pose : poses) {
            Translation2d translation = pose.getTranslation();
            arr[ndx + 0] = translation.getX();
            arr[ndx + 1] = translation.getY();
            arr[ndx + 2] = pose.getRotation().getDegrees();
            ndx += 3;
        }
        this.driveSubsystem = drivetrain;
        addRequirements(followCommand.getRequirements());
    }

    @Override
    public void initialize() {
        table.getEntry("traj").setDoubleArray(arr);
        followCommand.initialize();
    }

    @Override
    public void execute() {
        followCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return followCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        followCommand.end(interrupted);
        driveSubsystem.setControl(driveSubsystem.m_pathApplyRobotSpeeds.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }
}
