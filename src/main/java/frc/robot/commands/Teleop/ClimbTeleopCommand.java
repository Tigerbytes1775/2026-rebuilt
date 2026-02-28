package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbTeleopCommand extends Command{

    private final Climb climb;
    private final XboxController controller;

    private final double target;

    public ClimbTeleopCommand(Climb climb, XboxController controller) {
        addRequirements(climb);
        this.climb = climb;
        this.controller = controller;

        if (SmartDashboard.getNumber("Climb Target", -1) == -1) {
            SmartDashboard.putNumber("Climb Target", 0);
        }

        target = SmartDashboard.getNumber("Climb Target", 0);

    }



    @Override
    public void execute(){
        if (controller.getLeftBumperButton()){
            climb.setTarget(0); //go to bottom
        } else if (controller.getRightBumperButton()){
            climb.setTarget(target); //go to top
        }
    }

    @Override
    public void end(boolean interrupted) {
       climb.setMotors(0);
    }
    
}
