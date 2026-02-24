package frc.robot.subsystems.TurretSubsystems;

import java.util.function.DoubleSupplier;

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

    private PIDController pidController =  new PIDController(
            0.001,
            0.0001,
            0.0001
        );

    private final double strength = 1;


    public LazySusan() {}

    public void setMotors(double percent) {
        double power = strength * percent;
        motor.set(power);
    }

    public void setTarget(double angle) {

        double target = angle /(2*Math.PI);
        pidController.setSetpoint(target);
        
    }

    public double getRotation() {
        return encoder.getPosition() * gearRatio;
    }

    @Override
    public void periodic() {
        double percent = pidController.calculate(encoder.getPosition());
        SmartDashboard.putNumber("Turret Rotation Power", percent);
        setMotors(percent);
    }

}