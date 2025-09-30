package frc.robot.subsystems.Drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents a subsystem responsible for controlling the drive system of a
 * robot. This includes methods for controlling
 * the movement, managing swerve drive modules, tracking the robot's position
 * and orientation, and other related tasks.
 */
public interface SwerveIO extends Subsystem {
    // void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean isOpenLoop);

    // void setModuleStates(SwerveModuleState[] desiredStates);

    // ChassisSpeeds getMeasuredSpeeds();

    // Rotation2d getGyroYaw();

    Pose2d getPose();

    // default Rotation2d getHeading() {
    // return getPose().getRotation();
    // }

    // default void setHeading(Rotation2d heading) {
    // setPose(new Pose2d(getPose().getTranslation(), heading));
    // }

    // default void zeroHeading() {
    // setHeading(new Rotation2d());
    // }

    // void addVisionMeasurement(Pose2d visionRobotPose, double timeStampSeconds);
    // void addVisionMeasurement(Pose2d visionRobotPoseMeters, double
    // timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    
    void resetOdometry(Pose2d pose2d);
    void seedFieldCentric();
}