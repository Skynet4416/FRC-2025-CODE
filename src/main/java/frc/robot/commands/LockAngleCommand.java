package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.meth.Alliance;
import frc.robot.meth.Distance;

public class LockAngleCommand extends Command {
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<Double> angleSetter;
    private final Consumer<Boolean> manualOverride;
    private final double lineLength;
    private final double maxDistance;
    private final Pose2d[] centers;

    public LockAngleCommand(Supplier<Pose2d> poseSupplier, Pose2d[] centers, double lineLength, double maxDistance,
            Consumer<Double> angleSetter,
            Consumer<Boolean> manualOverride) {
        this.poseSupplier = poseSupplier;
        this.angleSetter = angleSetter;
        this.manualOverride = manualOverride;
        this.centers = centers;
        this.maxDistance = maxDistance;
        this.lineLength = lineLength;
    }

    @Override
    public void initialize() {
        manualOverride.accept(false);
    }

    @Override
    public void execute() {
        Pose2d closestCenter = Distance.isPointNearLinesSegment(poseSupplier.get().getTranslation(), centers,
                lineLength, maxDistance);

        if (closestCenter != null) {
            Rotation2d rotation = closestCenter.getRotation().rotateBy(Alliance.apply(Rotation2d.k180deg));
            angleSetter.accept(rotation.getRadians());
        }

    }

    @Override
    public void end(boolean interrupted) {
        angleSetter.accept((double) -999);
        manualOverride.accept(true);
    }
}
