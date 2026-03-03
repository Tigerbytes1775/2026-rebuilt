package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystems.Launch;
import frc.robot.subsystems.TurretSubsystems.Turret;
import frc.robot.Constants.Targets;
public class TurretTeleopCommand extends Command {

    private double[] customShot;
    private final Turret turret;
    private final XboxController controller;

    public TurretTeleopCommand(Turret turret, XboxController controller) {
        addRequirements(turret);
        this.turret = turret;
        this.controller = controller;
        SmartDashboard.setPersistent("Custom Shot");
        
        
    }

    @Override
    public void initialize() {

        if (SmartDashboard.getNumberArray("Custom Shot", new double[]{}).length == 0) {
            SmartDashboard.putNumberArray("Custom Shot", Targets.farShot);
        }
        
        customShot = SmartDashboard.getNumberArray("Custom Shot", Targets.farShot);

        turret.lazySusan.zeroEncoders();
    }


    @Override
    public void execute() {

        Launch launch = turret.launch;

        boolean shooting = true;
        if (controller.getAButton()) {
            //turret.launch(Targets.hub);
            launch.setMotors(0.2);
        } else if (controller.getXButton()) {
            //turret.launch(Targets.leftShot);
            launch.setMotors(0.4);
        } else if (controller.getBButton()) {
            //turret.launch(Targets.rightShot);
            launch.setMotors(0.7);
        } else if (controller.getYButton()) {
            //turret.launch(customShot);
            launch.setMotors(0.89);
        } else {
            turret.powerDownLaunch();
            shooting = false;  
        }
        //System.out.println(shooting);
        SmartDashboard.putBoolean("Shooting", shooting);

    }
    
}
