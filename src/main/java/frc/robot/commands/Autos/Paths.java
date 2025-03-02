package frc.robot.commands.Autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

public enum Paths {

    // LCS Trajectories
    LCS_REEF1LEFT("LCS-Reef1Left"),
    LCS_REEF1RIGHT("LCS-Reef1Right"),
    LCS_REEF2LEFT("LCS-Reef2Left"),
    LCS_REEF2RIGHT("LCS-Reef2Right"),
    LCS_REEF3LEFT("LCS-Reef3Left"),
    LCS_REEF3RIGHT("LCS-Reef3Right"),
    LCS_REEF4LEFT("LCS-Reef4Left"),
    LCS_REEF4RIGHT("LCS-Reef4Right"),
    LCS_REEF5LEFT("LCS-Reef5Left"),
    LCS_REEF5RIGHT("LCS-Reef5Right"),
    LCS_REEF6LEFT("LCS-Reef6Left"),
    LCS_REEF6RIGHT("LCS-Reef6Right"),

    // RCS Trajectories (Fixed to match actual filenames)
    RCS_REEF1LEFT("RCS-Reef1Left"),
    RCS_REEF1RIGHT("RCS-Reef1Right"),
    RCS_REEF2LEFT("RCS-Reef2Left"),
    RCS_REEF2RIGHT("RCS-Reef2Right"),
    RCS_REEF3LEFT("RCS-Reef3Left"),
    RCS_REEF3RIGHT("RCS-Reef3Right"),
    RCS_REEF4LEFT("RCS-Reef4Left"),
    RCS_REEF4RIGHT("RCS-Reef4Right"),
    RCS_REEF5LEFT("RCS-Reef5Left"),
    RCS_REEF5RIGHT("RCS-Reef5Right"),
    RCS_REEF6LEFT("RCS-Reef6Left"),
    RCS_REEF6RIGHT("RCS-Reef6Right"),

    // LCS Reef with suffix "-LCS"
    REEF1LEFT_LCS("Reef1Left-LCS"),
    REEF1RIGHT_LCS("Reef1Right-LCS"),
    REEF2LEFT_LCS("Reef2Left-LCS"),
    REEF2RIGHT_LCS("Reef2Right-LCS"),
    REEF3LEFT_LCS("Reef3Left-LCS"),
    REEF3RIGHT_LCS("Reef3Right-LCS"),
    REEF4LEFT_LCS("Reef4Left-LCS"),
    REEF4RIGHT_LCS("Reef4Right-LCS"),
    REEF5LEFT_LCS("Reef5Left-LCS"),
    REEF5RIGHT_LCS("Reef5Right-LCS"),
    REEF6LEFT_LCS("Reef6Left-LCS"),
    REEF6RIGHT_LCS("Reef6Right-LCS"),

    // RCS Reef with suffix "-RCS" (Fixed to match actual filenames)
    REEF1LEFT_RCS("Reef1Left-RCS"),
    REEF1RIGHT_RCS("Reef1Right-RCS"),
    REEF2LEFT_RCS("Reef2Left-RCS"),
    REEF2RIGHT_RCS("Reef2Right-RCS"),
    REEF3LEFT_RCS("Reef3Left-RCS"),
    REEF3RIGHT_RCS("Reef3Right-RCS"),
    REEF4LEFT_RCS("Reef4Left-RCS"),
    REEF4RIGHT_RCS("Reef4Right-RCS"),
    REEF5LEFT_RCS("Reef5Left-RCS"),
    REEF5RIGHT_RCS("Reef5Right-RCS"),
    REEF6LEFT_RCS("Reef6Left-RCS"),
    REEF6RIGHT_RCS("Reef6Right-RCS"),

    RCS_POINT("RCS_Point"),
    TEST("Test"),
    TEST1("Test (1)");


    private String fileName;
    private PathPlannerPath path;

    Paths(String fileName) {
        this.fileName = fileName;
        try {
            this.path = PathPlannerPath.fromChoreoTrajectory(fileName);
        } catch (Exception e) {
            System.out.println(e);
            System.out.println(fileName
            );
        }
    }

    public PathPlannerPath getPath() {
        return this.path;
    }

    public Command getCommand() {
        return AutoBuilder.followPath(this.getPath());
    }
}
