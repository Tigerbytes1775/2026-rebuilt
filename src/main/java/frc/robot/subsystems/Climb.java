package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    
    private final SparkMax climbMotor = new SparkMax(33, SparkLowLevel.MotorType.kBrushless);
    private final RelativeEncoder encoder = climbMotor.getEncoder(); 

    private final double climbStrength = 1;


    private PIDController pidController =  new PIDController(
            0.001,
            0.0001,
            0.0001
        );
    
    
    public Climb() {}

    public void setMotors(double percent) {
        double power = percent * climbStrength;  
    
        SmartDashboard.putNumber("Climb power(%)", -power);

        if (percent == 0){
            climbMotor.stopMotor();
        } else {
            climbMotor.set(power);
        }
        
    }
     
    
    public void setTarget(double height) {
        pidController.setSetpoint(height);
        
    }
   

    @Override
    public void periodic() {
        double percent = pidController.calculate(encoder.getPosition());
        setMotors(percent);
    }


    
}
