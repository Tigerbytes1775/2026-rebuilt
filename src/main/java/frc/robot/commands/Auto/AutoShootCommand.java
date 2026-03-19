package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.TurretSubsystems.Turret;

public class AutoShootCommand extends Command {

    private final Turret turret;
    private final double[] target;
    private final double commandTime;
    private final Timer timer = new Timer();

    public AutoShootCommand(Turret turret, double[] target, double commandTime) {
        addRequirements(turret);
        this.turret = turret;
        this.target = target;
        this.commandTime = commandTime;
    }


    @Override
    public void initialize(){
        timer.reset();
        timer.start();
    }

    @Override
    public void execute(){
        turret.launch(target);
    }

    @Override 
    public boolean isFinished() {
        return timer.get() >= commandTime;
    }

    @Override
    public void end(boolean interrupted) {
       turret.powerDownLaunch();
    }
    
}
