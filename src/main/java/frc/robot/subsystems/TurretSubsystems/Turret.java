package frc.robot.subsystems.TurretSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Util.Aimer;
import frc.robot.subsystems.SwerveSubsystem;

public class Turret extends SubsystemBase{


    //private final ShootingSimulator simulator = new ShootingSimulator();
    private final Aimer aimer = new Aimer();

    private final Launch launch;
    private final LazySusan lazySusan;
    private final SwerveSubsystem swerve;

    public Turret(Launch launch, LazySusan lazySusan, SwerveSubsystem swerve) {
        this.launch = launch;
        this.lazySusan = lazySusan;
        this.swerve = swerve;
    }

    
    public void shoot(double[] target) {
        Pose2d pose = swerve.getPose();
        ChassisSpeeds chassisSpeed = swerve.getRobotVelocity();
        double[] roboPos = {pose.getX(), pose.getY(),0};
        double[] roboVel = {chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, 0};

        

        double[] shotInfo = aimer.aimShot(launch.incline, roboPos, target, roboVel);
        double launchSpeed = shotInfo[1];
        double angle = shotInfo[2];

        

        launch.setLaunchSpeed(launchSpeed);
        lazySusan.setTarget(angle);
        



    }


    
}
