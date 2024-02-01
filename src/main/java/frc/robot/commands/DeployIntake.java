package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

//ACCOUNT FOR TIME OF FLIGHT WHEN WE GET/FINISH IMPORTANT STUFF
public class DeployIntake extends Command {

    BooleanSupplier deploying;
    GroundIntake intake;
    Shooter shooter;
    Climber elevator;

    public DeployIntake(GroundIntake Intake, Shooter m_Shooter, Climber m_Climber, BooleanSupplier Deploying) 
    {
        this.deploying = Deploying;
        this.intake = Intake;
        this.shooter = m_Shooter;
        this.elevator = m_Climber;

        addRequirements(Intake);
        addRequirements(m_Shooter);
        addRequirements(m_Climber);
    }

    public void execute() {
        //ADD OR STATEMENT FOR FLIGHT SENSOR
        if (deploying.getAsBoolean() == false) 
        {
            intake.setRollers(0);
            intake.toSetPoint(0);
        } 
        else 
        {   
            elevator.toSetPoint(0);
            shooter.runFeederWheels(0);
            shooter.runFlyWheels(0);
            shooter.toSetPoint(1);
            intake.setRollers(50);
            intake.toSetPoint(1);
        }
    }
}