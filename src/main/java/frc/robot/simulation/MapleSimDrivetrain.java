package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

// import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;

public class MapleSimDrivetrain {
    // private final Pigeon2SimState pigeonSim;
    // private final SimSwerveModule[] simModules;
    public final SwerveDriveSimulation mapleSimDrive;

    public MapleSimDrivetrain() {
        final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
                .withGyro(COTS.ofPigeon2())
                .withSwerveModule(COTS.ofMark4i(
                        DCMotor.getKrakenX60(1),
                        DCMotor.getFalcon500(1),
                        COTS.WHEELS.VEX_GRIP_V2.cof,
                        3)) // L3 Gear ratio
                // Configures the track length and track width (spacing between swerve modules)
                .withTrackLengthTrackWidth(Inches.of(15.2), Inches.of(15.2))
                .withBumperSize(Inches.of(26), Inches.of(26));

        this.mapleSimDrive = new SwerveDriveSimulation( driveTrainSimulationConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(mapleSimDrive);
    }
}
