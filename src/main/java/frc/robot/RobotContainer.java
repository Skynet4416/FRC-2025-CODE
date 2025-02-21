// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveMoveToAngleIncreament;
import frc.robot.commands.Elevator.ElevatorMoveToHeight;
import frc.robot.commands.Elevator.ElevatorResetLimitSwitch;
import frc.robot.commands.Intake.IntakeAtPercentage;
import frc.robot.commands.Intake.IntakeCoral;
import frc.robot.commands.Intake.IntakeDefault;
import frc.robot.commands.TurnToAngle;
import frc.robot.meth.Distance;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.Telemetry;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeState;
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
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    //    private final ClimbDeepSubsystem climbDeepSubsystem = new ClimbDeepSubsystem();
//    private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(new LimelightObserver[]{drivetrain});
    private final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired
    // top speed
    private final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
    // second max angular
    // velocity

    // private final AutoFactory autoFactory;
    // private final Autos autoRoutines;
//    private final AutoChooser autoChooser = new AutoChooser();
    // public final CommandSwerveDrivetrain drivetrain;
    private final Telemetry logger = new Telemetry(MAX_SPEED);
    private boolean manualOverride = true;
    private double wantedAngle = 0;
    private final Trigger coralStationTrigger = new Trigger(() -> Distance.isPointNearLinesSegment(new Pose2d().getTranslation(),
            new Pose2d[]{FieldConstants.CoralStation.leftCenterFace, FieldConstants.CoralStation.rightCenterFace},
            FieldConstants.CoralStation.stationLength, Constants.States.Intake.RADIUS_IN_METERS) != null);

    private final Trigger reefTrigger = new Trigger(() -> Distance.isPointNearLinesSegment(new Pose2d().getTranslation(),
            FieldConstants.Reef.centerFaces, FieldConstants.Reef.faceLength, Constants.States.Score.RADIUS_IN_METERS) != null);

    private final Trigger intakeModeTrigger = new Trigger(() -> this.getState() == RobotState.INTAKE);
    private final Trigger climbTrigger = new Trigger(() -> this.getState() == RobotState.CLIMB);
    private final Trigger scoreTrigger = new Trigger(() -> this.getState() == RobotState.SCORE);
    private final Trigger intakeEmpty = new Trigger(() -> intakeSubsystem.getState() == IntakeState.EMPTY);
    private boolean readyToScore = false;
    private final Trigger readyToScoreTrigger = new Trigger(() -> readyToScore);
    private final SlewRateLimiter slewRateLimiterx = new SlewRateLimiter(1);
    private final SlewRateLimiter slewRateLimitery = new SlewRateLimiter(1);
    private final SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(10);

    private final DoubleSupplier xSupplier = () -> slewRateLimiterx.calculate(deadband(-IO.driverController.getLeftY())) * MAX_SPEED;
    private final DoubleSupplier ySupplier = () -> slewRateLimitery.calculate(deadband(-IO.driverController.getLeftX())) * MAX_SPEED;
    private final DoubleSupplier rotationSupplier = () -> slewRateLimiterRotation.calculate(deadband(-IO.driverController.getRightX())) * MAX_ANGULAR_RATE;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        drivetrain.configureAutoBuilder();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
        // autoFactory = drivetrain.createAutoFactory();
        // autoRoutines = new Autos(autoFactory);

        // autoChooser.addRoutine("SimplePath", () ->
        // autoRoutines.getAutoRoutine("SimplePath"));
        // SmartDashboard.putData("Auto Chooser", autoChooser);
        // Configure the trigger bindings
        configureBindings();
    }

    public double deadband(double value) {
        if (Math.abs(value) > 0.1) {
            return value;
        }

        return 0;
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor
     * with an arbitrary predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * null null null null null null null null null null null null null null
     * null null     {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or null null null null null null null null null null
     * null null null null null null     {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drivetrain.registerTelemetry(logger::telemeterize);

        IO.mechanismController.a().onTrue(new InstantCommand(() -> state
                = RobotState.INTAKE));
        IO.mechanismController.b().onTrue(new InstantCommand(() -> state
                = RobotState.SCORE));
        IO.mechanismController.y().onTrue(new InstantCommand(() -> state
                = RobotState.CLIMB));

        intakeSubsystem.setDefaultCommand(new IntakeDefault(intakeSubsystem));

        elevatorSubsystem.setDefaultCommand(new ElevatorResetLimitSwitch(elevatorSubsystem));

        coralStationTrigger.and(intakeModeTrigger).and(intakeEmpty).whileTrue(new IntakeCoral(intakeSubsystem).alongWith(new ElevatorMoveToHeight(elevatorSubsystem, Constants.States.Intake.ELEVATOR_HEIGHT).andThen(new InstantCommand(() -> intakeSubsystem.moveMotor(Constants.States.Intake.INTAKE_PERCEHNTAGE)).raceWith(new WaitCommand(0.3)))));

        reefTrigger.and(scoreTrigger).whileTrue(new ElevatorMoveToHeight(elevatorSubsystem, Constants.States.Score.ELEVATOR_HEIGHT).andThen(new InstantCommand(() -> readyToScore = true)));

        reefTrigger.and(scoreTrigger).and(readyToScoreTrigger).and(IO.mechanismController.leftBumper()).onTrue(new IntakeAtPercentage(intakeSubsystem, Constants.States.Score.INTAKE_PERCNETAGE).raceWith(new WaitCommand(Constants.States.Score.INTAKE_TIME)).andThen(new InstantCommand(() -> intakeSubsystem.setState(IntakeState.EMPTY)
        )));

        climbTrigger.whileTrue(new ElevatorMoveToHeight(elevatorSubsystem, Constants.States.Climb.ELEVATOR_HEIGHT));
        IO.mechanismController.x().whileTrue(new TurnToAngle(drivetrain, edu.wpi.first.math.util.Units.degreesToRadians(0)));
//        intakeSubsystem.setDefaultCommand(new IntakeBasedOnStateCommand(intakeSubsystem, this::getState, () -> new Pose2d()));
//        IO.mechanismController.leftBumper().whileTrue(new IntakeShootCommand(intakeSubsystem, this::getState, () -> new Pose2d()).andThen(new InstantCommand(()
//                -> elevatorSubsystem.setIntendedState(ElevatorState.DOWN))));
//
//        elevatorSubsystem.setDefaultCommand(new ElevatorBasedOnStateCommand(elevatorSubsystem, this::getState, () -> new Pose2d()));
//        IO.mechanismController.leftBumper().whileTrue(new LegGoDownCommand(climbDeepSubsystem).andThen(new InstantCommand(()
//                -> elevatorSubsystem.setIntendedState(ElevatorState.DOWN))));
//
        drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xSupplier, ySupplier, rotationSupplier, () -> wantedAngle, () -> manualOverride));
        IO.driverController.leftBumper().onTrue(new DriveMoveToAngleIncreament(-30, (angle) -> wantedAngle = edu.wpi.first.math.util.Units.degreesToRadians(angle), (a) -> this.manualOverride = a, drivetrain));
        IO.driverController.rightBumper().onTrue(new DriveMoveToAngleIncreament(30, (angle) -> wantedAngle = edu.wpi.first.math.util.Units.degreesToRadians(angle), (a) -> this.manualOverride = a, drivetrain));

        //    IO.mechanismController.x().whileTrue(new ElevatorResetLimitSwitch(elevatorSubsystem));
        //    IO.mechanismController.a().whileTrue(new ElevatorMoveToHeight(elevatorSubsystem, 0.125));
    }

    // /**
    //  * Use this to pass the autonomous command to the main {@link Robot} class.
    //  *
    //  * @return the command to run in autonomous
    //  */
//     public Command getAutonomousCommand() {
//       return autoChooser.selectedCommand();
//     }
    public RobotState getState() {
        SmartDashboard.putString("robot state", String.valueOf(this.state));
        return this.state;

    }

    public boolean getManualOverride() {
        return this.manualOverride;
    }

    public void containerTrigger() {
        SmartDashboard.putBoolean("coral station trigger", coralStationTrigger.getAsBoolean());
        SmartDashboard.putBoolean("intake Mode trigger", intakeModeTrigger.getAsBoolean());
        SmartDashboard.putBoolean("intkae empty trigger", intakeEmpty.getAsBoolean());
        SmartDashboard.putBoolean("reef trigger", reefTrigger.getAsBoolean());
        SmartDashboard.putBoolean("climb trigger", climbTrigger.getAsBoolean());
        SmartDashboard.putBoolean("score trigger", scoreTrigger.getAsBoolean());
        SmartDashboard.putBoolean("ready to score trigger", readyToScoreTrigger.getAsBoolean());
        drivetrain.periodic();
    }
}
