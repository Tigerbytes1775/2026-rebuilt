package frc.robot.subsystems.TurretSubsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Aimer;

public class Turret extends SubsystemBase{


    //private final ShootingSimulator simulator = new ShootingSimulator();
    private final Aimer aimer = new Aimer();

    private final Launch launch;
    private final LazySusan lazySusan;
    

    public Turret(Launch launch, LazySusan lazySusan) {
        this.launch = launch;
        this.lazySusan = lazySusan;
    }

    
    public void shoot(double[] target) {
        double[] roboPos = {-2, 1, 0};// fix this shit
        double[] roboVel = {3, 6, 0};

        double[] shotInfo = aimer.aimShot(launch.incline, roboPos, target, roboVel);
        double launchSpeed = shotInfo[1];
        double angle = shotInfo[2];

        launch.setLaunchSpeed(launchSpeed);
        lazySusan.setTarget(angle);
        



    }


    
}
