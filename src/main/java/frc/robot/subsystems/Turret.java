package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Util.Aimer;
import frc.robot.Util.ShootingSimulator;

public class Turret extends SubsystemBase{

    private final PIDController pidController;


    private final ShootingSimulator simulator = new ShootingSimulator();
    private final Aimer aimer = new Aimer();


    private final TalonFX launchMotor1 = new TalonFX(5);
    private final TalonFX launchMotor2 = new TalonFX(6);

    private final double launchWheelRadius = 2 * 0.0254;//  2 inches converted to meters
    private final double launchGearRatio = 8;

    private final double incline = Math.toRadians(70);

    private final double launchKvRating = 505.44;// rpm/voltage 
    private final double launchStrengh = 1;


    private final SparkFlex rotationMotor = new SparkFlex(4, MotorType.kBrushless);
    private final RelativeEncoder rotationEncoder = rotationMotor.getEncoder(); 

    private final DoubleSupplier rotationSupplier = rotationEncoder::getPosition;


    private final double rotationStrength = 1;


    public Turret() {

        pidController = new PIDController(
            0.001,
            0.0001,
            0.0001
        );
    }



    public void setRotationMotors(double percent) {
        double power = rotationStrength * percent;
        rotationMotor.set(power);
    }

    public void setLaunchMotors(double percent) {
        double power = percent * launchStrengh;

        launchMotor1.set(power);
        launchMotor2.set(power);

        rotationEncoder.setPosition(0.25);
        
    }

    public void setLaunchRPM(double rpm) {

        double volts = rpm / launchKvRating;
        launchMotor1.setVoltage(volts);
        launchMotor1.setVoltage(volts);
    }

    public void setLaunchVelocity(double mps) {
        double rpm = (mps*60) / (2*Math.PI*launchWheelRadius) * launchGearRatio;
        setLaunchRPM(rpm);
    }

    public void setTargetDirection(double[] target) {

        double angle = Math.atan2(target[1], target[0]);
        pidController.setSetpoint(angle);
        
    }

    public void shoot(double[] target) {
        double[] roboPos = {-2, 1, 0};// fix this shit
        double[] roboVel = {3, 6, 0};

        aimer.aimShot(incline, roboPos, target, roboVel);
    }


    @Override
    public void periodic() {
        double power = pidController.calculate(rotationSupplier.getAsDouble());
        SmartDashboard.putNumber("Turret Rotation Power", power);
        setRotationMotors(power);
    }
}
