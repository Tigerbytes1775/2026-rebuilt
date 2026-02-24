package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbTeleopCommand extends Command{

    private final Climb climb;
    private final XboxController controller;

    public ClimbTeleopCommand(Climb climb, XboxController controller) {
        addRequirements(climb);
        this.climb = climb;
        this.controller = controller;
    }



    @Override
    public void execute(){
        if (controller.getLeftBumperButton()){
            climb.setTarget(0); //go to bottom
        } else if (controller.getRightBumperButton()){
            climb.setTarget(1); //go to top
        }
    }

    @Override
    public void end(boolean interrupted) {
       climb.setMotors(0);
    }
    
}
