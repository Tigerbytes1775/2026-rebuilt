package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    
    private final SparkMax motor = new SparkMax(10, SparkLowLevel.MotorType.kBrushless);

    
    
    public Intake() {

        SmartDashboard.setPersistent("Intake Strength");

        if(SmartDashboard.getNumber("Intake Strength", -10) == -10) {
            SmartDashboard.putNumber("Intake Strength", 0.5);
        }

        
    }

    public void setMotors(double percent) {

        double strength = SmartDashboard.getNumber("Intake Strength", 0.5);

        double power = percent * strength;

        SmartDashboard.putNumber("Intake power(%)", power);

        if (percent == 0){
            motor.stopMotor();
        } else {
            motor.set(-power);
        }
        
    }
    
}
