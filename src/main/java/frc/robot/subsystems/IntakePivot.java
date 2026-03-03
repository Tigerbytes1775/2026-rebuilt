package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePivot extends SubsystemBase {
    
    
    private final SparkMax intakePivotMotor = new SparkMax(28, SparkLowLevel.MotorType.kBrushless);

    
    
    
    public IntakePivot() {
        SmartDashboard.setPersistent("Intake Pivot Strength");
        if (SmartDashboard.getNumber("Intake Pivot Strength",0) == 0) {
            SmartDashboard.putNumber("Intake Pivot Strength", 0.5);
        }

    }

    public void setMotors(double percent) {

        double strength = SmartDashboard.getNumber("Intake Pivot Strength", 0.5);
        double power = percent * strength;

        SmartDashboard.putNumber("Intake Pivot power(%)", power);

        if (percent == 0){
            intakePivotMotor.stopMotor();
        } else {
            intakePivotMotor.set(power);
        }
        
    }
    
}
