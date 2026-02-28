package frc.robot.subsystems.TurretSubsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LazySusan extends SubsystemBase {

    private final double gearRatio = 36/200;

    private final SparkFlex motor = new SparkFlex(34, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder(); 
    private double[] PIDvalues;

    private final PIDController pidController; 

    private final double strength = 1;



    public LazySusan() {

        SmartDashboard.setPersistent("Lazy Susan PID");
        
        if (SmartDashboard.getNumberArray("Lazy Susan PID", new double[0]).length == 0) {
            SmartDashboard.putNumberArray("Lazy Susan PID", new double[]{0,0,0});
        }

        PIDvalues = SmartDashboard.getNumberArray("Lazy Susan PID", new double[]{0,0,0});
    
        pidController =  new PIDController(
            PIDvalues[0],
            PIDvalues[1],
            PIDvalues[2]
        );
    } 
    public void setMotors(double percent) {
        double power = strength * percent;
        motor.set(power);
        SmartDashboard.putNumber("Turret Rotation Power", power);
    }

    public void setTarget(double angle) {

        double target = angle /(2*Math.PI);
        pidController.setSetpoint(target);
        
    }

    public double getRotation() {
        return encoder.getPosition() * gearRatio;
    }

    public void zeroEncoders() {
        encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        double percent = pidController.calculate(encoder.getPosition());
        setMotors(percent);

       
        SmartDashboard.putNumber("Turret Rotation:(Radians)", getRotation());
        SmartDashboard.putNumber("Turret Rotation:(Degrees)", Math.toDegrees(getRotation()));


    }

}