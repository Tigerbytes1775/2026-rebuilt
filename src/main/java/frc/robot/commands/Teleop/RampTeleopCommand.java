package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Ramp;

public class RampTeleopCommand extends Command{

    private final Ramp ramp;
    private final XboxController controller;

    public RampTeleopCommand(Ramp ramp, XboxController controller) {
        addRequirements(ramp);
        this.ramp = ramp;
        this.controller = controller;
    }



    @Override
    public void execute(){

        if (controller.getRightTriggerAxis() >= 0.1) {
            ramp.setMotors(1);
        } else { 
            ramp.setMotors(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        ramp.setMotors(0);
    }
    
}

