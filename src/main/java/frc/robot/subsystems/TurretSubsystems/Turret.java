package frc.robot.subsystems.TurretSubsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Aimer;
import frc.robot.Util.ShootingSimulator;
import frc.robot.subsystems.Ramp;
import frc.robot.subsystems.SwerveSubsystem;

public class Turret extends SubsystemBase{

    // y  was 0.3429
    private final double[] robotToTurret = {0.1651,0.0,0.4064}; //x and y distance from center of robot to center of turret, and height of turret

    //private final ShootingSimulator simulator = new ShootingSimulator();
    private final Aimer aimer = new Aimer();
    private final ShootingSimulator sim = new ShootingSimulator();

    private final double minShootDistance = 0;
    private final double maxShootDistance = 100;

    private final Field2d field = new Field2d();
    private final FieldObject2d targetMarker = field.getObject("Target Marker");

    private final double moe = 0.2;

    public final Launch launch;
    public final Ramp ramp;
    public final LazySusan lazySusan;
    public final SwerveSubsystem swerve;
    private boolean wasReady = false;

    public Turret(Launch launch, LazySusan lazySusan, Ramp ramp, SwerveSubsystem swerve) {
        this.launch = launch;
        this.lazySusan = lazySusan;
        this.ramp = ramp;
        this.swerve = swerve;
    }


    

    public void aim(double target[]) {
        

       
        //System.out.println("Aiming Started");
        
        ChassisSpeeds chassisSpeed = swerve.getRobotVelocity();
        double[] robotVel = {chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond, 0};

        //System.out.println("Getting Shot Info");
        double[] shotInfo = aimer.aimShot(launch.incline, getTurretPos(), target, robotVel);
        //System.out.println(shotInfo);

        launch.setLaunchSpeed(shotInfo[1]);
        setGlobalAngle(2*Math.PI-shotInfo[2]);
       
    }


    public double[] getTurretPos() {
        double robotRotation = swerve.getPose().getRotation().getRadians();
        double[] robotPos = {swerve.getPose().getX(), swerve.getPose().getY(),0};

        double[] turretPos = {
            robotPos[0] + robotToTurret[0] * Math.cos(robotRotation),
            robotPos[1] + robotToTurret[1] * Math.sin(robotRotation),
            robotToTurret[2]
        };
        
        return turretPos;
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

        double[] robotVel = {swerve.getRobotVelocity().vxMetersPerSecond,swerve.getRobotVelocity().vyMetersPerSecond};

        aim(target);

        double distanceToTarget = getDistance(target, getTurretPos());
        //boolean goodDistance = distanceToTarget >= minShootDistance && distanceToTarget <= maxShootDistance;
        boolean ready = sim.checkShot(launch.getLaunchSpeed(), 2*Math.PI - getGlobalAngle() ,launch.incline,robotVel,getTurretPos(),target, moe);

        if(ready) {
            SmartDashboard.putBoolean("Shoot Success", true);
            ramp.setMotors(1);
            wasReady = true;
        } else {
            SmartDashboard.putBoolean("Shoot Success", false);
            if (wasReady) {
                wasReady = false;
                ramp.setMotors(0);
            }
            
        }

    }


    public void setGlobalAngle(double globalAngleTarget) {

        

        double robotAngle = swerve.getPose().getRotation().getRadians();
        double targetAngle = robotAngle + globalAngleTarget;

        while(targetAngle < 0) {
            targetAngle += 2* Math.PI;
        }

        targetAngle %= 2*Math.PI;

        lazySusan.setTarget(targetAngle);

    }

    public double getGlobalAngle() {
        double robotAngle = swerve.getPose().getRotation().getRadians();
        double susanAngle = lazySusan.getAngle();

        double globalAngle = susanAngle - robotAngle;
        

        while (globalAngle < 0) {
            globalAngle += 2*Math.PI;
        }
        globalAngle %= 2*Math.PI;

        return globalAngle;
    }


    public void powerDownLaunch() {
        launch.setTargetRPM(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Global Turret Angle", Math.toDegrees(getGlobalAngle()));
    }

}
