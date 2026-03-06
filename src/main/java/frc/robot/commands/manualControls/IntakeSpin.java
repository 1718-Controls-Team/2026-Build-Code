package frc.robot.commands.manualControls;



import frc.robot.subsystems.intakeFuel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


/** An example command that uses an example subsystem. */
public class IntakeSpin extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final intakeFuel m_intakeSubsystem;
    
    private boolean m_isFinished = false;
    private int m_stateMachine = 1;
    
    
      /**
       * Creates a new set-PowerCommand.
       *
       * @param subsystem The subsystem used by this command.
       */
    public IntakeSpin(intakeFuel intakeSubsystem) {
      m_intakeSubsystem = intakeSubsystem;
  
     
      // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
      
          
  }
   
    // Called when the command is initially scheduled.
    @Override
  public void initialize() {
        m_stateMachine = 1;
        m_isFinished = false;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
      m_intakeSubsystem.setIntakeSpinSpeed(Constants.kIntakeInSpeed);
    
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