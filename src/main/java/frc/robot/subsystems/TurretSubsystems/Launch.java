package frc.robot.subsystems.TurretSubsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launch extends SubsystemBase {

    private final TalonFX launchMotor1 = new TalonFX(5);
    private final TalonFX launchMotor2 = new TalonFX(6);

    public final double incline = Math.toRadians(70);

    private final double launchWheelRadius = 2 * 0.0254;//  2 inches converted to meters
    private final double launchGearRatio = 8;

    private final double launchKvRating = 505.44;// rpm/voltage 
    private final double launchStrengh = 1;

    public Launch() {}


    public void setLaunchMotors(double percent) {
        double power = percent * launchStrengh;

        launchMotor1.set(power);
        launchMotor2.set(power);
        
    }

    public void setLaunchRPM(double rpm) {

        double volts = rpm / launchKvRating;
        launchMotor1.setVoltage(volts);
        launchMotor1.setVoltage(volts);
    }

    public void setLaunchSpeed(double mps) {
        double rpm = (mps*60) / (2*Math.PI*launchWheelRadius) * launchGearRatio;
        setLaunchRPM(rpm);
    }



    
}
