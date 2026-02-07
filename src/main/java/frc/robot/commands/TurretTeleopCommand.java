package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystems.Turret;

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
        double[] hub = {4.035,4.03,1.1938}; //fix
        double[] leftShot = {2,2,0};
        double[] rightShot = {6,2,0};
        double[] farShot = {4.035,2,0};
        if (controller.getAButton()) {
            turret.shoot(hub);
        } else if (controller.getXButton()) {
            turret.shoot(leftShot);
        } else if (controller.getBButton()) {
            turret.shoot(rightShot);
        } else if (controller.getYButton()) {
            turret.shoot(farShot);
        }

    }
    
}
