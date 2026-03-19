package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.Intake;

public class IntakeAutoCommand extends Command{

    private final Intake intake;
    private final double commandTime;
    private final Timer timer = new Timer();
 

    public IntakeAutoCommand(Intake intake, double commandTime) {
        addRequirements(intake);
        this.intake = intake;
        this.commandTime = commandTime;
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override 
    public boolean isFinished() {
        return timer.get() >= commandTime;
    }

    @Override
    public void execute() {
        intake.setMotors(1);
    }

    @Override
    public void end(boolean interrupted) {
       intake.setMotors(0);
    }
    
}
