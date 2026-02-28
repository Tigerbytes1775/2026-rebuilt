package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbAutoCommand extends Command{

    private final Climb climb;
    private final double height;

    public ClimbAutoCommand(Climb climb, double height) {
        addRequirements(climb);
        this.climb = climb;
        this.height = height;
    }


    @Override
    public void initialize(){
        climb.setTarget(height);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
       climb.setMotors(0);
    }
    
}
