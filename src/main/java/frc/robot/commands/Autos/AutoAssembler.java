package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoAssembler {
    private Paths[] paths;
    public AutoAssembler(Paths[] paths){
        this.paths = paths;
    }
    
    public Command getCommand(){
        Command command = new InstantCommand();
        for(Paths path : paths){
            command.andThen(path.getCommand());
        }
        
        return command;
    }

}
