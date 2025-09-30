package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MapleSimSwerve extends SubsystemBase {
    private final SelfControlledSwerveDriveSimulation simulatedDrive;
    private final Field2d field2d;
    private final PIDController m_pathThetaController = new PIDController(Constants.Subsystems.Drive.Rotation.KP, 0, 0);
    
    public MapleSimSwerve() {
        final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(COTS.ofMark4(
                        DCMotor.getKrakenX60(1),
                        DCMotor.getFalcon500(1),
                        COTS.WHEELS.VEX_GRIP_V2.cof,
                        3))
                .withTrackLengthTrackWidth(Inches.of(15.22), Inches.of(15.22))
                .withBumperSize(Inches.of(26), Inches.of(26));

        this.simulatedDrive = new SelfControlledSwerveDriveSimulation(
                new SwerveDriveSimulation(config, new Pose2d(3, 3, new Rotation2d())));

        SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

        field2d = new Field2d();
        SmartDashboard.putData("simulation field", field2d);

        m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_pathThetaController.setTolerance(Units.degreesToRadians(1.5));
    }

    public void drive(double x, double y, double rotation) {
        this.simulatedDrive.runChassisSpeeds(
                new ChassisSpeeds(x, y, rotation),
                new Translation2d(),
                true,
                true);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedDrive.runSwerveStates(desiredStates);
    }

    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedDrive.getMeasuredSpeedsFieldRelative(true);
    }

    public Rotation2d getGyroYaw() {
        return simulatedDrive.getRawGyroAngle();
    }

    public Pose2d getPose() {
        return simulatedDrive.getOdometryEstimatedPose();
    }

    public void setPose(Pose2d pose) {
        simulatedDrive.setSimulationWorldPose(pose);
        simulatedDrive.resetOdometry(pose);
    }
    
    public double calculateRotation(double angleInRadians) {
        return m_pathThetaController.calculate(getPose().getRotation().getRadians(), angleInRadians);
    }

    public void periodic() {
        // update the odometry of the SimplifedSwerveSimulation instance
        simulatedDrive.periodic();

        // send simulation data to dashboard for testing
        field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
        field2d.getObject("odometry").setPose(getPose());
    }
}