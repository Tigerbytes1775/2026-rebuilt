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
        turret.lazySusan.zeroEncoders();
        
        
    }

    @Override
    public void initialize() {

        if (SmartDashboard.getNumberArray("Custom Shot", new double[]{}).length == 0) {
            SmartDashboard.putNumberArray("Custom Shot", Targets.farShot);
        }
        
        customShot = SmartDashboard.getNumberArray("Custom Shot", Targets.farShot);

        
    }


    @Override
    public void execute() {

        Launch launch = turret.launch;

        boolean shooting = true;
        if (controller.getAButton()) {
            //turret.launch(Targets.hub);
            launch.setTargetRPM(3000);
        } else if (controller.getXButton()) {
            //turret.launch(Targets.leftShot);
            launch.setTargetRPM(3500);
        } else if (controller.getBButton()) {
            //turret.launch(Targets.rightShot);
            launch.setTargetRPM(4500);
        } else if (controller.getYButton()) {
            //turret.launch(customShot);
            launch.setTargetRPM(5000);
        } else {
            turret.powerDownLaunch();
            shooting = false;  
        }

        double y = controller.getLeftY();
        double x = controller.getLeftX();
        double angle = Math.atan2(y,x ) - Math.PI/2;
        if (angle < 0) {
            angle += 2 * Math.PI;
        }

        if (Math.sqrt(x*x + y*y) >= 0.5) {
            turret.lazySusan.setTarget(angle);
        }

        //System.out.println(shooting);
        SmartDashboard.putBoolean("Shooting", shooting);

    }
    
}
