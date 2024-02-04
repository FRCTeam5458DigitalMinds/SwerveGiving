package frc.robot.commands;


import java.util.function.BooleanSupplier;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Timer;

public class Handoff extends Command
{

    Timer timer = new Timer();
    GroundIntake intake;
    Shooter shooter; 
    
    public Handoff(Shooter m_Shooter, GroundIntake m_Intake) 
    {
        this.intake = m_Intake;
        this.shooter = m_Shooter;

        addRequirements(m_Shooter);
        addRequirements(m_Intake);
        
    }

    public void initialize()
    {
        intake.toSetPoint(0);
        shooter.toSetPoint(1);
        intake.setRollers(-20);
        shooter.runFeederWheels(30);

        timer.restart();
    }

    public void execute()
    {
        isFinished();
    }
    
    public boolean isFinished()
    {
        if (timer.get() > 2)
        {
           intake.setRollers(0);
           shooter.runFeederWheels(0);

            
            return true;
        }
        return false;
    }

    
}
