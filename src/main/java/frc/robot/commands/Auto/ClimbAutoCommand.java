package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj.Timer;


public class ClimbAutoCommand extends Command{

    private final Climb climb;
    private final boolean climbing;
    private final double commandTime;
    private final Timer timer = new Timer();
    //if climbing false then descending
    public ClimbAutoCommand(Climb climb, boolean climbing, double commandTime) {
        addRequirements(climb);
        this.climb = climb;
        this.climbing = climbing;
        this.commandTime = commandTime;

    }


    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        if (climbing) {
            climb.climb();
        } else {
            climb.descend();
        }
    }

    @Override 
    public boolean isFinished() {
        return timer.get() >= commandTime;
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
       climb.setMotors(0);
    }
    
}
