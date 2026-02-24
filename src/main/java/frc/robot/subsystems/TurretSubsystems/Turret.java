package frc.robot.subsystems.TurretSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Util.Aimer;
import frc.robot.Util.ShootingSimulator;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.SwerveSubsystem;

public class Turret extends SubsystemBase{


    //private final ShootingSimulator simulator = new ShootingSimulator();
    private final Aimer aimer = new Aimer();

    private final Field2d field = new Field2d();
    private final FieldObject2d targetMarker = field.getObject("Target Marker");

    private final double marginOfError = 0.2;

    private final Launch launch;
    private final Ramp ramp;
    private final LazySusan lazySusan;
    private final SwerveSubsystem swerve;

    public Turret(Launch launch, LazySusan lazySusan, Ramp ramp, SwerveSubsystem swerve) {
        this.launch = launch;
        this.lazySusan = lazySusan;
        this.ramp = ramp;
        this.swerve = swerve;
    }

    public void aim(double target[]) {

        Pose2d pose = swerve.getPose();
        ChassisSpeeds chassisSpeed = swerve.getRobotVelocity();
        double[] roboPos = {pose.getX(), pose.getY(),0};
        double[] roboVel = {chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, 0};


        double[] shotInfo = aimer.aimShot(launch.incline, roboPos, target, roboVel);
        double launchSpeed = shotInfo[1];
        double angle = shotInfo[2] - pose.getRotation().getRadians();


        launch.setLaunchSpeed(launchSpeed);
        lazySusan.setTarget(angle);
    }

  
    
    public void launch(double[] target) {

        ShootingSimulator sim = new ShootingSimulator();


        targetMarker.setPose(target[0], target[1], new Rotation2d(0,0));
        SmartDashboard.putNumberArray("Turret Target", target);
        SmartDashboard.putData("Field", field);

        double robotRotation = swerve.getPose().getRotation().getRadians();

        double[] robotPos = {swerve.getPose().getX(), swerve.getPose().getY()};
        double[] robotVel = {swerve.getRobotVelocity().vxMetersPerSecond,swerve.getRobotVelocity().vyMetersPerSecond};

        aim(target);

        if(sim.checkShot(launch.getLaunchSpeed(), lazySusan.getRotation() - robotRotation,launch.incline,robotVel,robotPos,target,marginOfError)) {
            ramp.setMotors(1);
        } else {
            ramp.setMotors(0);
        }

    }

}
