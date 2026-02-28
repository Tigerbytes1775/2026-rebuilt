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
    private double[] PIDvalues;

    private final PIDController pidController; 
        
    public Climb() {

        SmartDashboard.setPersistent("Climb PID"); 

        if (SmartDashboard.getNumberArray("Climb PID", new double[0]).length == 0) {
            SmartDashboard.putNumberArray("Climb PID", new double[]{0,0,0});
        }
        
        PIDvalues = SmartDashboard.getNumberArray("Climb PID", new double[]{0,0,0});

        pidController =  new PIDController(
            PIDvalues[0],
            PIDvalues[1],
            PIDvalues[2]
        );
    }
        
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
