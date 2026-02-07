
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;

public class IntakePivotCommand extends Command{

    private final IntakePivot intakePivot;
    private final XboxController controller;

    public IntakePivotCommand(IntakePivot intakePivot, XboxController controller) {
        addRequirements(intakePivot);
        this.intakePivot = intakePivot;
        this.controller = controller;
    }



    @Override
    public void execute(){

        if (controller.getLeftY() > 0.1 || controller.getLeftY() < -0.1){
            intakePivot.setMotors(-controller.getLeftY());
        } else { 
            intakePivot.setMotors(0);
        }
    
    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.setMotors(0);
    }
    
}
