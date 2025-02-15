// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Elevator.ElevatorMoveToHeight;
import frc.robot.commands.Elevator.ElevatorMoveUpBySetPercentage;
import frc.robot.commands.Elevator.ElevatorResetLimitSwitch;
import frc.robot.subsystems.Drive.Telemetry;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

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
    // private final ClimbDeepSubsystem climbDeepSubsystem;
    private final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // private final AutoFactory autoFactory;
    // private final Autos autoRoutines;
    private final AutoChooser autoChooser = new AutoChooser();
    // public final CommandSwerveDrivetrain drivetrain;
    private final Telemetry logger = new Telemetry(MAX_SPEED);
    private boolean manualOverride = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and
     * commands.
     */
    public RobotContainer() {
        // drivetrain = TunerConstants.createDrivetrain();
        // autoFactory = drivetrain.createAutoFactory();
        // autoRoutines = new Autos(autoFactory);
        // climbDeepSubsystem = new ClimbDeepSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();

        // autoChooser.addRoutine("SimplePath", () -> autoRoutines.getAutoRoutine("SimplePath"));
        // SmartDashboard.putData("Auto Chooser", autoChooser);
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
        // drivetrain.registerTelemetry(logger::telemeterize);

//        IO.mechanismController.a().onTrue(new InstantCommand(() -> state = RobotState.INTAKE).alongWith(new InstantCommand(() -> elevatorSubsystem.setIntendedState(ElevatorState.UP))));
//        IO.mechanismController.b().onTrue(new InstantCommand(() -> state = RobotState.SCORE).alongWith(new InstantCommand(() -> elevatorSubsystem.setIntendedState(ElevatorState.UP))));
//        IO.mechanismController.y().onTrue(new InstantCommand(() -> state = RobotState.CLIMB).alongWith(new InstantCommand(() -> elevatorSubsystem.setIntendedState(ElevatorState.UP))));
//
//        intakeSubsystem.setDefaultCommand(new IntakeBasedOnStateCommand(intakeSubsystem, this::getState, () -> new Pose2d()));
//        IO.mechanismController.leftBumper().whileTrue(new IntakeShootCommand(intakeSubsystem, this::getState, () -> new Pose2d()).andThen(new InstantCommand(() -> elevatorSubsystem.setIntendedState(ElevatorState.DOWN))));
//
//        elevatorSubsystem.setDefaultCommand(new ElevatorBasedOnStateCommand(elevatorSubsystem, this::getState, () -> this.drivetrain.getState().Pose));
//        IO.mechanismController.leftBumper().whileTrue(new LegGoDownCommand(climbDeepSubsystem).andThen(new ElevatorMoveToHeight(elevatorSubsystem, Constants.States.Climb.ELEVATOR_DOWN).alongWith(new InstantCommand(() -> elevatorSubsystem.setIntendedState(ElevatorState.DOWN)))));
//
//        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, this::getState, () -> -IO.driverController.getLeftY() * MAX_SPEED, () -> -IO.driverController.getLeftX() * MAX_SPEED, () -> -IO.driverController.getRightX() * MAX_ANGULAR_RATE, this::getManualOverride));
//        IO.driverController.rightBumper().onTrue(new InstantCommand(() -> this.manualOverride = true));
//        IO.driverController.rightBumper().onFalse(new InstantCommand(() -> this.manualOverride = false));

        IO.mechanismController.leftBumper().whileTrue(new ElevatorMoveUpBySetPercentage(elevatorSubsystem));
        IO.mechanismController.rightBumper().whileTrue(new ElevatorResetLimitSwitch(elevatorSubsystem));
        IO.mechanismController.a().whileTrue(new ElevatorMoveToHeight(elevatorSubsystem, 0.1));
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
