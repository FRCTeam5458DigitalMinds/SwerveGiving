package frc.robot.commands;

import java.util.function.BooleanSupplier;

import javax.lang.model.util.ElementScanner14;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer; 

public class AutoShoot extends Command
{
   
    Timer timer = new Timer();
    
    Shooter shooter;
    GroundIntake intake;
    double custyTIME;
    int MODE;

    private double elevator_point;
    //commented out bc no reasonable way to determine if we are there without doing math we would be "skipping"
    //  private double podium_degrees = 33.333;
    //  private double subwoofer_degrees = 65.662787;

    private double distance;
    private double degrees;
    
    public AutoShoot(Shooter m_Shooter, GroundIntake m_Intake, int mode) 
    {
        this.shooter = m_Shooter;
        this.intake = m_Intake;

        this.MODE = mode;

        addRequirements(m_Shooter);
        addRequirements(m_Intake);

    }

    public void initialize()
    {
       //  this.elevator_point = climber.getInches();
        
        if (MODE == 0)
        {
            shooter.runFlyWheels(-95);
            shooter.toSetPoint(0);
            shooter.runFeederWheels(0);
            
    
            intake.setRollers(0);
            custyTIME = 1;
        }
        if (MODE == 1)
        {
            shooter.runFlyWheels(-95);
            shooter.toCustomSetpoint(17);
            shooter.runFeederWheels(0);
            custyTIME = 2.5;
        }
        if (MODE == 2)
        {
            shooter.runFlyWheels(0);
            shooter.runFeederWheels(0);
            intake.setRollers(0);
            custyTIME = 0.1;
        }

        timer.restart();
    }

    public void execute()
    {
        isFinished();
    }

    public boolean isFinished()
    {
        if (timer.get() > custyTIME)
        {
            if (MODE < 2)
            {
                intake.setRollers(-50);
                shooter.runFeederWheels(85);
            }
            return true;
        }

        return false;
    }    
}
//tehee