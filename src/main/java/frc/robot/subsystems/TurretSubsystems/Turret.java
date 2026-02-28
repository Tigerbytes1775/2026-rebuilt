package frc.robot.subsystems.TurretSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.Aimer;
import frc.robot.Util.ShootingSimulator;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.SwerveSubsystem;

public class Turret extends SubsystemBase{


    private final double[] robotToTurret = {0,0,0.22};

    //private final ShootingSimulator simulator = new ShootingSimulator();
    private final Aimer aimer = new Aimer();
    private final ShootingSimulator sim = new ShootingSimulator();

    private final double minShootDistance = 0;
    private final double maxShootDistance = 100;

    private final Field2d field = new Field2d();
    private final FieldObject2d targetMarker = field.getObject("Target Marker");

    private final double acceptableMOE = 0.2;//MOE is margin of error

    public final Launch launch;
    public final Ramp ramp;
    public final LazySusan lazySusan;
    private final SwerveSubsystem swerve;

    public Turret(Launch launch, LazySusan lazySusan, Ramp ramp, SwerveSubsystem swerve) {
        this.launch = launch;
        this.lazySusan = lazySusan;
        this.ramp = ramp;
        this.swerve = swerve;
    }


    

    public void aim(double target[]) {
        //System.out.println("Aiming Started");
        Pose2d pose = swerve.getPose();
        ChassisSpeeds chassisSpeed = swerve.getRobotVelocity();
        double[] robotPos = {pose.getX(), pose.getY(),0};
        double[] turretPos = {robotPos[0] + robotToTurret[0], robotPos[1] + robotToTurret[1], robotPos[2] + robotToTurret[2]};
        double[] robotVel = {chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, 0};

        //System.out.println("Getting Shot Info");
        double[] shotInfo = aimer.aimShot(launch.incline, turretPos, target, robotVel);
        //System.out.println(shotInfo);
        double launchSpeed = shotInfo[1];
        double angle = shotInfo[2] - pose.getRotation().getRadians();


        launch.setLaunchSpeed(launchSpeed);
        lazySusan.setTarget(angle);
    }

    private double getDistance(double[] a, double[] b) {
        double[] diff = {a[0] - b[0], a[1] - b[1]};
        return Math.sqrt(diff[0] * diff[0] + diff[1] + diff[1]);
    }
    /*
     * Aims and launches balls at target. Won't shoot if it projects it will miss.
     */
    public void launch(double[] target) {

        targetMarker.setPose(target[0], target[1], new Rotation2d(0));
        SmartDashboard.putNumberArray("Turret Target", target);
        SmartDashboard.putData("Field", field);

        double robotRotation = swerve.getPose().getRotation().getRadians();

        double[] robotPos = {swerve.getPose().getX(), swerve.getPose().getY(),0};
        double[] turretPos = {robotPos[0] + robotToTurret[0], robotPos[1] + robotToTurret[1], robotPos[2] + robotToTurret[2]};
        double[] robotVel = {swerve.getRobotVelocity().vxMetersPerSecond,swerve.getRobotVelocity().vyMetersPerSecond};

        aim(target);
        
        double distanceToTarget = getDistance(target, turretPos);
        boolean ready = sim.checkShot(launch.getLaunchSpeed(), lazySusan.getRotation() - robotRotation,launch.incline,robotVel,turretPos,target,acceptableMOE);
        if(distanceToTarget >= minShootDistance && distanceToTarget <= maxShootDistance && ready) {
            ramp.setMotors(1);
        } else {
            ramp.setMotors(0);
        }

    }

    public void powerDownLaunch() {
        launch.setTargetRPM(0);
    }

}
