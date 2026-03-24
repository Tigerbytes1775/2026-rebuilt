package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystems.Launch;
import frc.robot.subsystems.TurretSubsystems.Turret;
import frc.robot.Constants;
import frc.robot.Constants.Targets;

import java.lang.annotation.Target;
import java.util.*;
public class TurretTeleopCommand extends Command {


    private List<Double> rmpList = new ArrayList<>();
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
            turret.launch(Targets.hub);
            //launch.setTargetRPM(2250);
        } else if (controller.getBButton()) {
            turret.launch(Targets.leftShot);
            //launch.setTargetRPM(2750);
        } else if (controller.getXButton()) {
            turret.launch(Targets.rightShot);
            //launch.setTargetRPM(3250);
        } else if (controller.getYButton()) {
            turret.launch(customShot);
            //launch.setTargetRPM(3750);{
        } else {

            turret.powerDownLaunch();
            shooting = false;  
            if (turret.wasReady) {
                turret.ramp.setMotors(0);
                turret.wasReady = false;
            }

            if (controller.getPOV() != -1) {
                if (controller.getPOV() == 0) {
                    launch.setMotors(1);
                } else {
                    launch.setTargetRPM(launch.finalToCurrentRpm(2250));

                }
                
            } 
                //turret.setGlobalAngle(Math.atan2(Targets.hub[1],Targets.hub[0]));
                
            

            double y = controller.getRightY();
            double x = controller.getRightX();
           

            if (Math.sqrt(x*x + y*y) >= 0.5) {

                double angle = Math.atan2(x,-y) - Math.PI;
                
                while (angle < 0) {
                    angle += Math.PI*2;
                }

                if(!Constants.isBlue) {
                    angle += Math.PI;
                }

                angle %= 2*Math.PI;


                //SmartDashboard.putBoolean("Manual Lazy Susan", true);
                turret.setGlobalAngle(angle);
            }
        }

            
    

        SmartDashboard.putNumber("Global Turret Angle", Math.toDegrees(turret.getGlobalAngle()));

        //System.out.println(shooting);
        SmartDashboard.putBoolean("Shooting", shooting);
        //if(controller.getLeftBumperButton()) {
        //    rmpList.add(turret.launch.getRPM());
        //} 
        //if (controller.getRightBumperButton()) {
        //    rmpList.clear();
        //}
        //try {
        //    SmartDashboard.putNumber("RPM Minimum", Collections.min(rmpList));
        //} catch(Exception e) {
        //    SmartDashboard.putNumber("RPM Minimum", 0);
        //}
       

    }
    
}
