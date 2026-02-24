package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    
    
    private final SparkMax climbMotor = new SparkMax(33, SparkLowLevel.MotorType.kBrushless);

    private final double climbStrength = 0.5;
    
    
    public Climb() {}

    public void setMotors(double percent) {
        double power = percent * climbStrength;

        SmartDashboard.putNumber("Climb power(%)", power);

        if (percent == 0){
            climbMotor.stopMotor();
        } else {
            climbMotor.set(power);
        }
        
    }
    
}
