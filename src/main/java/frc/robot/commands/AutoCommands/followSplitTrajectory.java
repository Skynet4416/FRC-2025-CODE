package frc.robot.commands.AutoCommands;

import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.utils.ShakBarzLib;
import choreo.trajectory.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class followSplitTrajectory extends Command {
    private final CommandSwerveDrivetrain drive;
    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController rotationPID;
    private final Timer timer;
    private final double tolerance = 0.05;
    private final String pathName;
    private double startTime;

    private final Optional<Trajectory<SwerveSample>> trajectory;

    public followSplitTrajectory(CommandSwerveDrivetrain drive, String pathName) {
        this.drive = drive;
        this.xPID = drive.getXPathController();
        this.yPID = drive.getYPathController();
        this.rotationPID = drive.getRotationPathController();
        this.trajectory = Choreo.loadTrajectory(pathName);
        this.pathName = pathName;
        this.timer = new Timer();
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xPID.reset();
        yPID.reset();
        rotationPID.reset();
        timer.restart();
        startTime = ShakBarzLib.getClosestPoint(pathName, drive.getPose());
    }

    @Override
    public void execute() {
        if (trajectory.isPresent()) {
            Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get() + startTime, isRedAlliance());

            if (sample.isPresent()) {
                this.drive.followTrajectory(sample.get());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return this.drive.getState().Pose.getTranslation().getDistance(
                this.trajectory.get().getFinalSample(isRedAlliance()).get().getPose().getTranslation()) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        drive.setControl(drive.m_pathApplyRobotSpeeds.withSpeeds(zeroSpeeds));
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
}
