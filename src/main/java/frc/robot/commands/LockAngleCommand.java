package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class LockAngleCommand extends Command {
    private final Supplier<Pose2d> poseSupplier;
    private final DoubleSupplier angleSupplier;
    private final Consumer<Double> angleSetter;
    private final Consumer<Boolean> manualOverride;

    public LockAngleCommand(Supplier<Pose2d> poseSupplier, DoubleSupplier angleSupplier, Consumer<Double> angleSetter,
            Consumer<Boolean> manualOverride) {
        this.poseSupplier = poseSupplier;
        this.angleSetter = angleSetter;
        this.manualOverride = manualOverride;
        this.angleSupplier = angleSupplier;

    }

    @Override
    public void initialize() {
        Pose2d pose = poseSupplier.get();
        double suppliedAngle = this.angleSupplier.getAsDouble();
        double wantedAngle = pose != null
                ? pose.getRotation().plus(Rotation2d.k180deg).getDegrees()
                : suppliedAngle;
        angleSetter.accept(wantedAngle);
        if (wantedAngle != suppliedAngle) {
            manualOverride.accept(false);
        }

    }
}
