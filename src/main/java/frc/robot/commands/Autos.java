package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class Autos {
    private final AutoFactory m_factory;

    public Autos(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine getAutoRoutine(String routineName) {
        final AutoRoutine routine = m_factory.newRoutine(routineName);
        final AutoTrajectory simplePath = routine.trajectory(routineName);

        routine.active().onTrue(
                simplePath.resetOdometry()
                        .andThen(simplePath.cmd())
        );
        return routine;
    }
}