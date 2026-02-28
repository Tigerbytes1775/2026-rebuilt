package frc.robot.commands.Teleop;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class VisionTeleopCommand extends Command{

    private final Vision vision;

    private final BooleanSupplier dPadDown;
    
    public VisionTeleopCommand(Vision vision, BooleanSupplier dPadDown) {
        addRequirements(vision);
        this.dPadDown = dPadDown;
        this.vision = vision;


    }

    @Override
    public void initialize() {
        vision.visionEnabled = true;
    }

    @Override
    public void execute() {
        if(vision.visionEnabled) {
            vision.addVisionMeasurement();
        }

        if(dPadDown.getAsBoolean()) {
            vision.visionEnabled = false;
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
    
}