package frc.robot.commands;

import java.util.function.BooleanSupplier;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

//ACCOUNT FOR TIME OF FLIGHT WHEN WE GET/FINISH IMPORTANT STUFF
public class DeployIntake extends Command {

    BooleanSupplier deploying;
    GroundIntake intake;
    Shooter shooter;

    public DeployIntake(GroundIntake Intake, Shooter m_Shooter, BooleanSupplier Deploying) 
    {
        this.deploying = Deploying;
        this.intake = Intake;
        this.shooter = m_Shooter;

        addRequirements(Intake);
        addRequirements(m_Shooter);
    }

    public void execute() {
        if (deploying.getAsBoolean() == false) 
        {
            intake.setRollers(false);
            intake.toSetPoint(0);
        } 
        else 
        {   
            shooter.toSetPoint(1);
            intake.setRollers(true);
            intake.toSetPoint(1);
        }
    }
}