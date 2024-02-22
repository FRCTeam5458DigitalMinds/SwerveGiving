
package frc.robot.commands;

import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class Eject extends Command
{
   
    GroundIntake intake;
    int MODE;
    
    //commented out bc no reasonable way to determine if we are there without doing math we would be "skipping"
    //  private double podium_degrees = 33.333;
    //  private double subwoofer_degrees = 65.662787;

    public Eject(GroundIntake m_Intake, int MODE) 
    {
        this.intake = m_Intake;

        addRequirements(m_Intake);

    }

    public void initialize()
    {      
        if (MODE == 0)
        {     
            intake.toSetPoint(3);
            intake.setRollers(-80);
        }
        else
        {
            intake.setRollers(0);
            intake.toSetPoint(0);
        }
        isFinished();
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }

    /*
    public void execute() 
    {

        //GET RID OF THIS AND MAKE FILE SPECIFIC    
        isFinished();
    }

    public boolean isFinished()
    {

        if (timer.get() > 1)
        {
            intake.setRollers(-80);
            return true;
        }
    return false;
    } */
}    
