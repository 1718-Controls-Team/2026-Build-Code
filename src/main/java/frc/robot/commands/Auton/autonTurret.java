package frc.robot.commands.Auton;



import frc.robot.subsystems.turretHood;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class autonTurret extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final turretHood m_turretSubsystem;
  
    private boolean m_isFinished = false;
    private Timer erikaTimer = new Timer();
  
  
    /**
     * Creates a new set-PowerCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public autonTurret(turretHood turretSubsystem) {
      m_turretSubsystem = turretSubsystem;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(turretSubsystem);
      
          
    }
   
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_isFinished = false;
        erikaTimer.reset();
        erikaTimer.start();
    }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (erikaTimer.get() <= 0.25) {
      m_turretSubsystem.setTurretMotorR(-2);
      m_turretSubsystem.setTurretMotorL(2);
      m_isFinished = true;
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
