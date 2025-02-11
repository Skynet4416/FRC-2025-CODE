package frc.robot.autonomus;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Autos {
    private AutoChooser autoChooser = new AutoChooser();
    private AutoFactory factory;

    public Autos(AutoFactory factory) {
        File deployDriectory = Filesystem.getDeployDirectory();

        for (File file : deployDriectory.listFiles()) {
            autoChooser.addRoutine(file.getName(), () -> this.buildRoutine(file.getName()));
        }

        this.factory = factory;
    }

    public AutoRoutine buildRoutine(String routineName) {
        final AutoRoutine routine = factory.newRoutine(routineName);
        final AutoTrajectory simplePath = routine.trajectory(routineName);

        routine.active().onTrue(
                simplePath.resetOdometry()
                        .andThen(simplePath.cmd()));
        return routine;
    }

    public AutoChooser getAutoChooser() {
        return autoChooser;
    }

    public void addCommandToAutos(String key, Command command) {
        NamedCommands.registerCommand(key, command);
    }
}