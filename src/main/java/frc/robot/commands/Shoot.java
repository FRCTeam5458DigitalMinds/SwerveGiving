package frc.robot.commands;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command
{
    Climber climber;
    Shooter shooter;
    GroundIntake intake;

    private int elevator_point;

    public Shoot(Climber m_Climber, Shooter m_Shooter, GroundIntake m_Intake) 
    {
        this.climber = m_Climber;
        this.shooter = m_Shooter;
        this.intake = m_Intake;

        this.elevator_point = climber.getStage();

        addRequirements(m_Climber);
        addRequirements(m_Shooter);
        addRequirements(m_Intake);
    }

    public void execute() {
        if (elevator_point == 0)
        {
            intake.setRollers(-50);
            //CALL AUTOMATIC LIMELIGHTSHOOTING HERE!!!!
        } 
        else if (elevator_point == 1)
        {
            shooter.runFeederWheels(95);
            shooter.runFlyWheels(80);
        }

    }
}