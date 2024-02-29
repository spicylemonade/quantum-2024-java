package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {
    public RobotContainer(){
        // Subsystem initialization

        // Register Named Commands
        //replace print with actual shooting code
        NamedCommands.registerCommand("autoBalance", Commands.print("Passed marker 1"));
        
        

        // Do all other initialization
        //configureButtonBindings();

    }
    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in GUIthe 
        PathPlannerPath path = PathPlannerPath.fromPathFile("shoot");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    }
    
}
