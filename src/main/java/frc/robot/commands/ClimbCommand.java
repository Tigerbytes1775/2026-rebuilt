package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends Command{

    private final Climb climb;
    private final XboxController controller;

    public ClimbCommand(Climb climb, XboxController controller) {
        addRequirements(climb);
        this.climb = climb;
        this.controller = controller;
    }



    @Override
    public void execute(){

        if (controller.getRightY() >= 0.2) {
            climb.setMotors(1);
        } else if (controller.getRightY() <= -0.2 ) {
            climb.setMotors(-1);
        } else {
            climb.setMotors(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
       climb.setMotors(0);
    }
    
}
