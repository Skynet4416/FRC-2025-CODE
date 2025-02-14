// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.Elevator.ElevatorBasedOnStateCommand;
import frc.robot.commands.Elevator.ElevatorMoveToHeight;
import frc.robot.commands.Intake.IntakeBasedOnStateCommand;
import frc.robot.commands.Intake.IntakeShootCommand;
import frc.robot.commands.LegGoDownCommand;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Leg.ClimbDeepSubsystem;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.Telemetry;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private RobotState state = RobotState.NONE;
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ClimbDeepSubsystem climbDeepSubsystem;
    private double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final AutoFactory autoFactory;
    private final Autos autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    public final CommandSwerveDrivetrain drivetrain;
    private final Telemetry logger = new Telemetry(MAX_SPEED);
    private boolean manualOverride = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        drivetrain = TunerConstants.createDrivetrain();
        autoFactory = drivetrain.createAutoFactory();
        autoRoutines = new Autos(autoFactory);
        climbDeepSubsystem = new ClimbDeepSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();

        autoChooser.addRoutine("SimplePath", () -> autoRoutines.getAutoRoutine("SimplePath"));
        SmartDashboard.putData("Auto Chooser", autoChooser);
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
     * with an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for      {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or      {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        IO.mechanismController.a().onTrue(new InstantCommand(() -> state = RobotState.INTAKE));
        IO.mechanismController.b().onTrue(new InstantCommand(() -> state = RobotState.SCORE));
        IO.mechanismController.y().onTrue(new InstantCommand(() -> state = RobotState.CLIMB));

        intakeSubsystem.setDefaultCommand(new IntakeBasedOnStateCommand(intakeSubsystem, this::getState, () -> this.drivetrain.getState().Pose));
        IO.mechanismController.leftBumper().whileTrue(new IntakeShootCommand(intakeSubsystem, this::getState, () -> this.drivetrain.getState().Pose));

        elevatorSubsystem.setDefaultCommand(new ElevatorBasedOnStateCommand(elevatorSubsystem, this::getState, () -> this.drivetrain.getState().Pose));
        IO.mechanismController.leftBumper().whileTrue(new LegGoDownCommand(climbDeepSubsystem).andThen(new ElevatorMoveToHeight(elevatorSubsystem, Constants.States.Climb.ELEVATOR_DOWN)));

        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, this::getState, () -> -IO.driverController.getLeftX(), () -> -IO.driverController.getLeftY(), () -> -IO.driverController.getRightX(), this::getManualOverride));
        IO.driverController.rightBumper().onTrue(new InstantCommand(() -> this.manualOverride = true));
        IO.driverController.rightBumper().onFalse(new InstantCommand(() -> this.manualOverride = false));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.selectedCommand();
    }

    public RobotState getState() {
        return this.state;
    }

    public boolean getManualOverride() {
        return this.manualOverride;
    }
}
