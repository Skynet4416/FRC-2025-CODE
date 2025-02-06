package frc.robot;

import java.lang.reflect.MalformedParametersException;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

public class AutoRoutines {
    private final AutoFactory m_factory;

    public AutoRoutines(AutoFactory factory) {
        m_factory = factory;
    }

    public AutoRoutine simplePathAuto() {
        final AutoRoutine routine = m_factory.newRoutine("SimplePath Auto");
        final AutoTrajectory simplePath = routine.trajectory("test");

        routine.active().onTrue(
            simplePath.resetOdometry()
                .andThen(simplePath.cmd())
        );
        return routine;
    }
    public AutoRoutine teleopRCS(){
        final AutoRoutine routine = m_factory.newRoutine("RCSPath Auto");
        final AutoTrajectory pathRCS = routine.trajectory("Reef6_TO_RCS");

        routine.active().onTrue(
            pathRCS.resetOdometry()
                .andThen(pathRCS.cmd())
        );
        return routine;
    }
    public AutoRoutine teleopLCS(){
        final AutoRoutine routine = m_factory.newRoutine("LCSPath Auto");
        final AutoTrajectory pathLCS = routine.trajectory("Reef6_To_LCS");

        routine.active().onTrue(
            pathLCS.resetOdometry()
                .andThen(pathLCS.cmd())
        );
        return routine;
    }
}
