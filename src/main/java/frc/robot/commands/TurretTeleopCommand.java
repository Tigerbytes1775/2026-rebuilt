package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystems.Turret;
import frc.robot.Constants.Targets;
public class TurretTeleopCommand extends Command {

    
    private final Turret turret;
    private final XboxController controller;

    public TurretTeleopCommand(Turret turret, XboxController controller) {
        addRequirements(turret);
        this.turret = turret;
        this.controller = controller;
    }


    @Override
    public void execute() {


        boolean shooting = true;
        if (controller.getAButton()) {
            turret.launch(Targets.hub);
        } else if (controller.getXButton()) {
            turret.launch(Targets.leftShot);
        } else if (controller.getBButton()) {
            turret.launch(Targets.rightShot);
        } else if (controller.getYButton()) {
            turret.launch(Targets.farShot);
        } else {
           shooting = false;  
        }
        //System.out.println(shooting);
        SmartDashboard.putBoolean("Shooting", shooting);

    }
    
}
