package frc.robot.commands;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class DriveMoveToAngleIncreament extends Command {

    private Consumer<Double> angleSetter;
    private Consumer<Boolean> manualOVerrideSetter;
    private CommandSwerveDrivetrain drivetrain;
    private double increament;

    public DriveMoveToAngleIncreament(double increament, Consumer<Double> angleSetter, Consumer<Boolean> manualOverrideConsumer, CommandSwerveDrivetrain drivetrain) {
        this.angleSetter = angleSetter;
        this.manualOVerrideSetter = manualOverrideConsumer;
        this.drivetrain = drivetrain;
        this.increament = increament;
    }

    @Override
    public void initialize() {
        manualOVerrideSetter.accept(false);
        double absincreament = Math.abs(increament);
        angleSetter.accept(Math.floor(drivetrain.getGyroRotationInDegrees() / absincreament) * absincreament + increament);

    }

    @Override
    public boolean isFinished() {
        return drivetrain.atRotationSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        manualOVerrideSetter.accept(true);
    }

}
