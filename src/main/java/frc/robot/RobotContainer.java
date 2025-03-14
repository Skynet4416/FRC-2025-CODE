// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.States;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveMoveToAngleIncreament;
import frc.robot.commands.LockAngleCommand;
import frc.robot.commands.AutoCommands.Forwards;
import frc.robot.commands.Autos.TrajCommnd;
import frc.robot.commands.Elevator.ElevatorMoveToHeight;
import frc.robot.commands.Elevator.ElevatorResetLimitSwitch;
import frc.robot.commands.Elevator.ElevatorResetLimitSwitchEnd;
import frc.robot.commands.Intake.IntakeAtPercentage;
import frc.robot.commands.Intake.IntakeCoral;
import frc.robot.commands.Intake.IntakeDefault;
import frc.robot.meth.Distance;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.Telemetry;
import frc.robot.subsystems.Drive.TunerConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Vision.LimelightObserver;
import frc.robot.subsystems.Vision.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        private final AutoFactory autoFactory;

        private RobotState state = RobotState.NONE;
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
        // private final ClimbDeepSubsystem climbDeepSubsystem = new
        // ClimbDeepSubsystem();
        private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem(
                        new LimelightObserver[] { drivetrain });
        private final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts
                                                                                                   // desired
        // top speed
        private final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation
                                                                                                  // per
        // second max angular
        // velocity

        // private final AutoFactory autoFactory;
        // private final Autos autoRoutines;
        private final AutoChooser autoChooser = new AutoChooser();
        // public final CommandSwerveDrivetrain drivetrain;

        private final Telemetry logger = new Telemetry(MAX_SPEED);
        private boolean manualOverride = true;
        private double wantedAngle = -999;
        private final Trigger coralStationTrigger = new Trigger(() -> Distance.isPointNearLinesSegment(
                        new Pose2d().getTranslation(),
                        new Pose2d[] { FieldConstants.CoralStation.leftCenterFace,
                                        FieldConstants.CoralStation.rightCenterFace },
                        FieldConstants.CoralStation.stationLength, Constants.States.Intake.RADIUS_IN_METERS) != null);

        private final Trigger reefTrigger = new Trigger(
                        () -> Distance.isPointNearLinesSegment(getPose().getTranslation(),
                                        FieldConstants.Reef.centerFaces, FieldConstants.Reef.faceLength,
                                        Constants.States.Score.RADIUS_IN_METERS) != null);

        private final Trigger intakeModeTrigger = new Trigger(() -> this.getState() == RobotState.INTAKE);
        private final Trigger climbTrigger = new Trigger(() -> this.getState() == RobotState.CLIMB);
        private final Trigger scoreTrigger = new Trigger(() -> this.getState() == RobotState.SCORE);
        private final Trigger intakeEmpty = new Trigger(() -> intakeSubsystem.getState() == IntakeState.EMPTY);
        private boolean readyToScore = false;
        private final Trigger readyToScoreTrigger = new Trigger(() -> readyToScore);
        private final SlewRateLimiter slewRateLimiterx = new SlewRateLimiter(6);
        private final SlewRateLimiter slewRateLimitery = new SlewRateLimiter(6);
        private final SlewRateLimiter slewRateLimiterRotation = new SlewRateLimiter(10);

        private final DoubleSupplier xSupplier = () -> slewRateLimiterx
                        .calculate(deadband(-IO.driverController.getLeftY())
                                        * MathUtil.clamp((1 - IO.driverController.getRightTriggerAxis()), 0.1, 1)
                                        * MAX_SPEED);
        private final DoubleSupplier ySupplier = () -> slewRateLimitery
                        .calculate(deadband(-IO.driverController.getLeftX())
                                        * MathUtil.clamp((1 - IO.driverController.getRightTriggerAxis()), 0.1, 1)
                                        * MAX_SPEED);
        private final DoubleSupplier rotationSupplier = () -> slewRateLimiterRotation
                        .calculate(deadband(-IO.driverController.getRightX())
                                        * MathUtil.clamp((1 - IO.driverController.getLeftTriggerAxis()), 0.1, 1))
                        * MAX_ANGULAR_RATE;
        private final Command autoCommand;

        public RobotContainer() {

                autoFactory = new AutoFactory(
                                drivetrain::getPose, // A function that returns the current robot pose
                                drivetrain::resetOdometry, // A function that resets the current robot pose to the
                                                           // provided Pose2d
                                drivetrain::followTrajectory, // The drive subsystem trajectory follower
                                true, // If alliance flipping should be enabled
                                drivetrain // The drive subsystem
                );

                autoChooser.addCmd("left", this::pickupAndRizzAutoSide);
                autoChooser.addCmd("middle", this::middleCommand);
                autoChooser.addCmd("right", this::rightCommand);
                autoChooser.addCmd("forward", this::forward);
                autoChooser.addCmd("middle complicated", this::pickupAndRizzAuto);
                SmartDashboard.putData("auto", autoChooser);
                configureBindings();
                autoCommand = pickupAndRizzAutoSide();
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
         * null null {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or null null null null null null null null null null
         * null null null null null null
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {
                drivetrain.registerTelemetry(logger::telemeterize);

                IO.mechanismController.a().onTrue(new InstantCommand(() -> state = RobotState.INTAKE));
                IO.mechanismController.b().onTrue(new InstantCommand(() -> state = RobotState.SCORE));
                IO.mechanismController.y().onTrue(new InstantCommand(() -> state = RobotState.CLIMB));
                IO.mechanismController.x().whileTrue(new IntakeAtPercentage(intakeSubsystem, -1));

                intakeSubsystem.setDefaultCommand(new IntakeDefault(intakeSubsystem));

                elevatorSubsystem.setDefaultCommand(new ElevatorResetLimitSwitch(elevatorSubsystem));

                coralStationTrigger.and(intakeModeTrigger).and(intakeEmpty).whileTrue(
                                new IntakeCoral(intakeSubsystem)
                                                .alongWith(new ElevatorMoveToHeight(elevatorSubsystem,
                                                                Constants.States.Intake.ELEVATOR_HEIGHT)
                                                                .andThen(
                                                                                new InstantCommand(
                                                                                                () -> {
                                                                                                        intakeSubsystem.moveMotor(
                                                                                                                        Constants.States.Intake.INTAKE_PERCEHNTAGE);
                                                                                                })
                                                                                                .raceWith(new WaitCommand(
                                                                                                                0.3)))));
                reefTrigger.and(scoreTrigger)
                                .whileTrue(new LockAngleCommand(this::getPose, FieldConstants.Reef.centerFaces,
                                                FieldConstants.Reef.faceLength, States.Score.RADIUS_IN_METERS,
                                                this::angleSetter,
                                                this::manualOverrideSetter));

                reefTrigger.and(scoreTrigger)
                                .whileTrue(new ElevatorMoveToHeight(elevatorSubsystem,
                                                Constants.States.Score.ELEVATOR_HEIGHT)
                                                .andThen(new InstantCommand(() -> readyToScore = true)));

                reefTrigger.and(scoreTrigger).and(readyToScoreTrigger).and(IO.mechanismController.leftBumper())
                                .onTrue(new IntakeAtPercentage(intakeSubsystem,
                                                Constants.States.Score.INTAKE_PERCNETAGE)
                                                .raceWith(new WaitCommand(Constants.States.Score.INTAKE_TIME))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))));

                drivetrain.setDefaultCommand(new DriveCommand(drivetrain, xSupplier, ySupplier, rotationSupplier,
                                () -> wantedAngle, () -> manualOverride));

                IO.driverController.rightBumper().onTrue(new InstantCommand(() -> manualOverride = true));
                IO.driverController.rightBumper().onFalse(new InstantCommand(() -> manualOverride = false));

                IO.driverController.b()
                                .whileTrue(new WaitCommand(0.1)
                                                .andThen(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())));

                IO.driverController.x()
                                .whileTrue(new WaitCommand(0.1)
                                                .andThen(new InstantCommand(
                                                                () -> drivetrain.resetOdometry(new Pose2d()))));

        }

        // /**
        // * Use this to pass the autonomous command to the main {@link Robot} class.
        // *
        // * @return the command to run in autonomous
        // */
        public Command getAutonomousCommand() {
                return autoChooser.selectedCommand();
        }

        public RobotState getState() {
                SmartDashboard.putString("robot state", String.valueOf(this.state));
                return this.state;

        }

        public Pose2d getPose() {
                return this.drivetrain.getPose();
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

        // shaki ops
        public CommandSwerveDrivetrain getDrive() {
                return drivetrain;
        }

        public Command getIntakeCommand() {
                return new IntakeCoral(intakeSubsystem).deadlineFor(new ElevatorMoveToHeight(elevatorSubsystem,
                                Constants.States.Intake.ELEVATOR_HEIGHT)).raceWith(new WaitCommand(3))
                                .andThen(new ElevatorResetLimitSwitchEnd(
                                                elevatorSubsystem));

        }

        public Command middleCommand() {
                return Commands.sequence(autoFactory.resetOdometry("Line-to-Reef4"), //
                                new TrajCommnd(autoFactory, "Line-to-Reef4", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -1)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))));
        }

        public Command rightCommand() {
                return Commands.sequence(autoFactory.resetOdometry("Line-to-Reef5"),
                                new TrajCommnd(autoFactory, "Line-to-Reef5", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -1)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))));
        }

        public Command forward() {
                return new SequentialCommandGroup(
                                new Forwards(drivetrain).raceWith(new WaitCommand(6)),
                                new IntakeAtPercentage(intakeSubsystem, -1)
                                                .raceWith(new WaitCommand(Constants.States.Score.INTAKE_TIME * 2)),
                                new InstantCommand(() -> intakeSubsystem.setState(IntakeState.EMPTY)));
        }

        public Command pickupAndRizzAuto() {
                return Commands.sequence(
                                autoFactory.resetOdometry("Line-to-Reef4"), //
                                new TrajCommnd(autoFactory, "Line-to-Reef4", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -1)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))),
                                new TrajCommnd(autoFactory, "Reef4Right-LCS", drivetrain),
                                getIntakeCommand(),
                                new TrajCommnd(autoFactory, "LCS-Reef2Left", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -.5)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))),
                                new TrajCommnd(autoFactory, "Reef2Left-LCS", drivetrain),
                                getIntakeCommand(),
                                new TrajCommnd(autoFactory, "LCS-Reef2Right", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -.5)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))),
                                new TrajCommnd(autoFactory, "Reef2Right-LCS", drivetrain),
                                getIntakeCommand(),
                                new TrajCommnd(autoFactory, "LCS-Reef2Left", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -.5)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))));
        }

        public Command pickupAndRizzAutoSide() {
                return Commands.sequence(
                                autoFactory.resetOdometry("Line-to-Reef3"), //
                                new TrajCommnd(autoFactory, "Line-to-Reef3", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -1)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))),
                                new TrajCommnd(autoFactory, "Reef3Right-LCS", drivetrain),
                                getIntakeCommand(),
                                new TrajCommnd(autoFactory, "LCS-Reef2Left", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -.5)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))),
                                new TrajCommnd(autoFactory, "Reef2Left-LCS", drivetrain),
                                getIntakeCommand(),
                                new TrajCommnd(autoFactory, "LCS-Reef2Right", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -.5)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))),
                                new TrajCommnd(autoFactory, "Reef2Right-LCS", drivetrain),
                                getIntakeCommand(),
                                new TrajCommnd(autoFactory, "LCS-Reef2Left", drivetrain),
                                new IntakeAtPercentage(intakeSubsystem, -.5)
                                                .raceWith(new WaitCommand(0.25))
                                                .andThen(new InstantCommand(
                                                                () -> intakeSubsystem.setState(IntakeState.EMPTY))));
        }

        public void lockAngle(Pose2d[] centers, double distance, double maxDistance) {
                Pose2d closestCenter = Distance.isPointNearLinesSegment(getPose().getTranslation(), centers, distance,
                                maxDistance);
                this.wantedAngle = closestCenter.getRotation().rotateBy(Rotation2d.k180deg).getRadians();
                this.manualOverride = true;

        }

        public void angleSetter(double angle) {
                this.wantedAngle = angle;
        }

        public void manualOverrideSetter(boolean manualOverride) {
                this.manualOverride = manualOverride;
        }
}
