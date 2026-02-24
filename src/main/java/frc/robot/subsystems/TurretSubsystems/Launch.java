package frc.robot.subsystems.TurretSubsystems;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launch extends SubsystemBase {

    private final SparkFlex launchMotor = new SparkFlex(32, MotorType.kBrushless);
    

    public final double incline = Math.toRadians(75);

    private final double launchWheelRadius = 2 * 0.0254;//  2 inches converted to meters
    private final double gearRatio = 1;

    private final double launchKvRating = 505.44;// rpm/voltage 
    private final double launchStrengh = 1;

    public Launch() {}


    public void setLaunchMotors(double percent) {
        double power = percent * launchStrengh;

        launchMotor.set(power);

        
    }

    public void setLaunchRPM(double rpm) {

        double volts = rpm / launchKvRating;
        launchMotor.setVoltage(volts);
      
    }

    public void setLaunchSpeed(double mps) {
        double rpm = (mps*60) / (2*Math.PI*launchWheelRadius) * gearRatio;
        setLaunchRPM(rpm);
    }

    public double getRPM() {
        return launchMotor.getEncoder().getVelocity();
    }

    public double getLaunchSpeed() {
        return getRPM()*2*Math.PI*launchWheelRadius/(60*gearRatio);
    }

    
}
