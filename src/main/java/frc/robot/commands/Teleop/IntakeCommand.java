package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command{

    private final Intake intake;
    private final XboxController controller;

    public IntakeCommand(Intake intake, XboxController controller) {
        addRequirements(intake);
        this.intake = intake;
        this.controller = controller;
    }



    @Override
    public void execute(){

        if (controller.getLeftTriggerAxis() >= 0.1) {
            intake.setMotors(1);
        } else { 
            intake.setMotors(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setMotors(0);
    }
    
}
