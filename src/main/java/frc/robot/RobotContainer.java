package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.Drive.DRotation;
import static frc.robot.Constants.Drive.DriveDeadband;
import static frc.robot.Constants.Drive.IRotation;
import static frc.robot.Constants.Drive.MaxAngularRatePercentage;
import static frc.robot.Constants.Drive.MaxSpeedPercentage;
import static frc.robot.Constants.Drive.PRotation;
import static frc.robot.Constants.Drive.RotationDeadband;
import static frc.robot.Constants.Drive.SnapDriveDeadband;
import static frc.robot.Constants.Drive.SnapRotationDeadband;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.quixlib.viz.Link2d;
import frc.quixlib.viz.Viz2d;
import frc.robot.commands.CoralEleUp;
import frc.robot.commands.CoralMoveScore;
import frc.robot.commands.CoralMoveStow;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.Moveup;
import frc.robot.commands.MoveupArm;
import frc.robot.commands.StowArm;
import frc.robot.generated.Telemetry;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DrivetrainExtra;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    public static double MaxSpeed = MaxSpeedPercentage*(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(MaxAngularRatePercentage).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final CommandXboxController driver = new CommandXboxController(0);
    public static final CommandXboxController operator = new CommandXboxController(1);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(TunerConstants.DrivetrainConstants,250, Constants.Vision.ODOM_STD_DEV, Constants.Vision.kSingleTagStdDevs, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DriveDeadband).withRotationalDeadband(MaxAngularRate * RotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    public static final SwerveRequest.FieldCentric driveAuto = new SwerveRequest.FieldCentric()
            .withDeadband(SnapDriveDeadband).withRotationalDeadband(SnapRotationDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public final SwerveRequest.FieldCentricFacingAngle angle = new SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(MaxSpeed * SnapRotationDeadband).withRotationalDeadband(MaxAngularRate * SnapRotationDeadband) // Add a deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors 
         //  .withSteerRequestType(SteerRequestType.MotionMagicExpo); // Use motion magic control for steer motors

    private PowerDistribution powerDistribution = new PowerDistribution();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;


Map<String, Command> robotCommands  = new HashMap<String, Command>();



private static  final Viz2d robotViz =
      new Viz2d("Robot Viz", Units.inchesToMeters(80.0), Units.inchesToMeters(120.0), 1.0);

private static  final Link2d chassisViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Chassis",
              Units.inchesToMeters(29.0),
              30.0,
              new Color("#FAB604"),
              new Transform2d(Constants.Viz.xOffset, Units.inchesToMeters(3.0), new Rotation2d())));

  // Elevator viz
  private  static final Link2d elevatorFrameViz =
      robotViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Base",
              Constants.Viz.elevatorBaseLength,
              4.0,
              Color.kGreen,
              new Transform2d(
                  Constants.Viz.elevatorBaseX,
                  Constants.Viz.elevatorBaseY,
                  Constants.Viz.elevatorAngle)));
  private static  final Link2d elevatorCarriageViz =
      elevatorFrameViz.addLink(
          new Link2d(
              robotViz,
              "Elevator Carriage",
              Constants.Viz.elevatorCarriageLength,
              6.0,
              Color.kLightGreen));
// Intake viz
// private static final Link2d intakeArmViz =
// robotViz.addLink(
//     new Link2d(robotViz, "Intake Arm", Constants.Viz.intakeArmLength, 10.0, Color.kBlue));
// private static final Link2d intakeRollerViz =
// intakeArmViz.addLink(
//     new Link2d(robotViz, "Intake Roller", Units.inchesToMeters(1.0), 10.0, Color.kLightBlue));


private static final Link2d ArmArmViz =
elevatorCarriageViz.addLink(
        new Link2d(robotViz, "Arm Arm", Constants.Viz.ArmArmLength, 10, Color.kRed));
private static final Link2d ArmWristViz =
ArmArmViz.addLink(
        new Link2d(robotViz, "Arm Wrist", Constants.Viz.ArmWristLength, 10, Color.kOrange));
private static final Link2d ArmWheelViz =
ArmWristViz.addLink(
        new Link2d(robotViz, "Arm Wheel", Units.inchesToMeters(2.0), 10, Color.kCoral));



        public static final ElevatorSubsystem elevator = new ElevatorSubsystem(elevatorCarriageViz);
        //public static final IntakeSubsystem intake = new IntakeSubsystem(intakeArmViz, intakeRollerViz);
        public static final ArmSubsystem arm = new ArmSubsystem(ArmArmViz,ArmWristViz,ArmWheelViz);






    public RobotContainer() {
        // Starts recording to data log
        DataLogManager.start();
        // Record both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog());

        //robotCommands.put("IntakePiece", new IntakeAlgae(intake,1).withTimeout(2.5));
        robotCommands.put("CoralMoveScore", new CoralMoveScore(elevator, arm));
        robotCommands.put("CoralMoveStow", new CoralMoveStow(elevator, arm));
        robotCommands.put("IntakeCoral", new IntakeCoral(elevator, arm));
        robotCommands.put("StowArm", new StowArm(elevator, arm));
        robotCommands.put("L4", Commands.runOnce(() -> RobotContainer.elevator.setHeightLocation(4)));

    
        NamedCommands.registerCommands(robotCommands);




        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto", autoChooser);
        angle.HeadingController.setPID( PRotation,  IRotation , DRotation);
        configureBindings();

    


        SmartDashboard.putData(
        "Gyro",
        builder -> {
          builder.setSmartDashboardType("Gyro");
          builder.addDoubleProperty("Value", () -> drivetrain.getPigeon2().getYaw().getValueAsDouble(), null);
        });
         // SmartDashboard.putNumber("Time",Timer.getMatchTime());
          SmartDashboard.putNumber("Time",DriverStation.getMatchTime());
          if(Constants.ExtraInfo){
          SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());
          SmartDashboard.putNumber("CAN",RobotController.getCANStatus().percentBusUtilization * 100.0);
          SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData("Power Distribution Panel", powerDistribution);
          }

        //PathfindingCommand.warmupCommand().ignoringDisable(true).schedule();;
        
    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        driver.leftBumper().whileTrue(new ParallelCommandGroup(new CoralMoveScore(elevator, arm), DrivetrainExtra.pathfindingCommand(true,true)));
        driver.rightBumper().whileTrue(new ParallelCommandGroup(new CoralMoveScore(elevator, arm), DrivetrainExtra.pathfindingCommand(false,true)));
        driver.leftBumper().onFalse(new CoralMoveStow(elevator, arm));
        driver.rightBumper().onFalse(new CoralMoveStow(elevator, arm));

        //driver.y().whileTrue(new ClimbSet(climber));
        //driver.x().whileTrue(pathfindingtofollowCommand());
        driver.leftTrigger().or(() -> (RobotContainer.elevator.getHeight() >= Constants.Elevator.SlowmodeHeight) && !DriverStation.isAutonomous()).whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed * Constants.Drive.SlowSpeedPercentage) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate * Constants.Drive.SlowRotPercentage) // Drive counterclockwise with negative X (left)
        ));
        driver.rightTrigger().whileTrue(new IntakeCoral(elevator, arm));
       // driver.leftTrigger().whileTrue(new IntakeAlgae(intake, 0));
        driver.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        

        // driver.y().whileTrue(drivetrain.applyRequest(() ->
        //     angle.withVelocityX(-driver.getLeftY() * MaxSpeed)
        //     .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //     .withTargetDirection(new Rotation2d(Math.toRadians(0))))
        // );
        


       // driver.a().whileTrue(new AlgeaWack(elevator, arm));
       
       driver.a().whileTrue(new MoveupArm(1,elevator,arm)); 
       driver.b().whileTrue(new MoveupArm(2,elevator,arm)); 
       driver.y().whileTrue(new Moveup(elevator));


       operator.y().whileTrue(new CoralEleUp(elevator));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a - single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        

        operator.rightBumper().whileTrue(new StowArm(elevator, arm));
        operator.start().whileTrue(drivetrain.applyRequest(() ->
        drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));


        //drivetrain.registerTelemetry(logger::telemeterize);
    }


// --------------------------------------------------------- Commands --------------------------------------------------------- 


    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }

   
}
//Abhi was here
