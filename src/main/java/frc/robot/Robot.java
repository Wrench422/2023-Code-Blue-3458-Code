// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import com.revrobotics.ColorSensorV3;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static final String placeAndGrab = "Place and Grab";
  private static final String TwoPieceScore = "Score Two";
  private static final String placeAndBalance = "Place and Balance";
  private static final String placeAndMobilize = "Place and Mobilize";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  //Declare all motors, solenoids, and joystick numbers here for easy access later, use prefix "c" for channel
  //These are all created first as variables so when the code is done, it is easy to come here and change the
  //numbers so all the motors etc. are on the right ports.

  //Drivetrain
  private static final int cLeftFrontDrive = 1;
private static final int cLeftRearDrive = 3;
private static final int cRightFrontDrive = 2;
private static final int cRightRearDrive = 4;
private MecanumDrive m_Drive;

//Other motors
private static final int cWristAngler = 5;

//Joysticks
private static final int cDriverStick = 0;
private static final int cOperatorStick = 1;
private Joystick m_driveStick;
private Joystick m_operatorStick;
//private final I2C.Port i2cPort = I2C.Port.kOnboard;

private double p = 0.0005;
private double i = 0.05;
private double d = 0.0;
private double p1 = 0.1;
private double i1 = 0.0;
private double d1 = 0.0;
//private double ks = 0.0;
//private double kg = 0.0;
//private double kv = 0.0;
private double bottomWrist = 00;
private double middleWrist = 540;
private double UpperWrist = 820;
private double pickUpPos = 600;
private double playStation = 1050;
private double sliderPickup = 140;
private boolean limeLightMove = false;
private boolean autoLevel = false;
private boolean compressorOn = true;
private boolean clawClosed = true;



private final DoubleSolenoid BottomArm = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 0, 1);
private final DoubleSolenoid UpperArm = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 2, 3);
//private final DoubleSolenoid LeftClaw = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 4, 5);
private final DoubleSolenoid RightClaw = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 4, 5);
private final DoubleSolenoid Brakes = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, 6, 7);
//private final DoubleSolenoid LimelightLifter = new DoubleSolenoid(2, PneumaticsModuleType.CTREPCM, 2, 3);
private final Compressor airCompressor = new Compressor(PneumaticsModuleType.CTREPCM);

public WPI_VictorSPX WristAngler = new WPI_VictorSPX(cWristAngler);

public Encoder encoder = new Encoder(3, 2);

private final PIDController wristController = new PIDController(p, i, d);
private final PIDController autoLeveler = new PIDController(p1, i1, d1);
private final PIDController limelightLR = new PIDController(p1, i1, d1);
private final PIDController limelightFB = new PIDController(p1, i1, d1);
private final PIDController rotateAuto = new PIDController(p1, i1, d1);
//private final ArmFeedforward wristFeedforward = new ArmFeedforward(ks, kg, kv);

private static final ADIS16470_IMU gyro = new ADIS16470_IMU();
private final I2C.Port i2cPort = I2C.Port.kOnboard;
private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

private final Timer m_timer = new Timer();

private CANSparkMax frontLeftDrive = new CANSparkMax(cLeftFrontDrive, MotorType.kBrushless);
private CANSparkMax rearLeftDrive = new CANSparkMax(cLeftRearDrive, MotorType.kBrushless);
private CANSparkMax frontRightDrive = new CANSparkMax(cRightFrontDrive, MotorType.kBrushless);
private CANSparkMax rearRightDrive = new CANSparkMax(cRightRearDrive, MotorType.kBrushless);
//private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
//private final DigitalInput wristUpright = new DigitalInput(1);
//private final DigitalInput wristFlat = new DigitalInput(0);


 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   * This encludes declaring all objects, setting up the auto selcetions, 
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Place and Grab", placeAndGrab);
    m_chooser.addOption("Place Two", TwoPieceScore);
    m_chooser.addOption("Place and Balance", placeAndBalance);
    m_chooser.addOption("Place and Mobilize", placeAndMobilize);
    SmartDashboard.putData("Auto choices", m_chooser);
  
    //Create all motors, solenoids, joysticks, etc. here using the variables created before
    
    //Set up motors for drivetrain
    // CANSparkMax frontLeftDrive = new CANSparkMax(cLeftFrontDrive, MotorType.kBrushless);
    // CANSparkMax rearLeftDrive = new CANSparkMax(cLeftRearDrive, MotorType.kBrushless);
    // CANSparkMax frontRightDrive = new CANSparkMax(cRightFrontDrive, MotorType.kBrushless);
    // CANSparkMax rearRightDrive = new CANSparkMax(cRightRearDrive, MotorType.kBrushless);
    rearLeftDrive.setInverted(false); //invert these motors so they go the right way
    rearRightDrive.setInverted(true);
    frontRightDrive.setInverted(true);
    
    //Set up the drivetrain as a mecanum drive
    m_Drive = new MecanumDrive(frontLeftDrive, rearLeftDrive, frontRightDrive, rearRightDrive);
    
   //Create and name the joysticks
    m_driveStick = new Joystick(cDriverStick);
    m_operatorStick = new Joystick(cOperatorStick);

    wristController.setTolerance(30);
    limelightLR.setTolerance(1);
    limelightFB.setTolerance(1);
    rotateAuto.setTolerance(3);
    autoLeveler.setTolerance(3);

    WristAngler.setNeutralMode(NeutralMode.Brake);
    encoder.setReverseDirection(true);
    encoder.reset();

    //encoder.setDistancePerPulse(4.0/360.0);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    encoder.reset();
    
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", m_autoSelected);
    System.out.println("Auto selected: " + m_autoSelected);
    

    Brakes.set(Value.kReverse);
    m_timer.reset();
    m_timer.start();
    gyro.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      
      case TwoPieceScore:
      if(m_timer.get() < 0.2){
        UpperArm.set(Value.kForward);
        BottomArm.set(Value.kForward);
                }
      if(m_timer.get() > 0.4 && m_timer.get() < 2){
        WristAngler.set(wristController.calculate(encoder.get(), 870));
      }
      if(m_timer.get() > 2 && m_timer.get() < 2.3){
        WristAngler.set(0.0);
      m_Drive.driveCartesian(0, 0.2, 0);  }      
      if(m_timer.get() > 2.3 && m_timer.get() < 2.4){
        RightClaw.set(Value.kForward);
        clawClosed = false;
      }
      if(m_timer.get() > 2.4 && m_timer.get() < 6.1){
        m_Drive.driveCartesian(0, -0.33, 0);
        WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
        UpperArm.set(Value.kReverse);
        BottomArm.set(Value.kReverse);
      }
      if(m_timer.get() > 6.1 && m_timer.get() < 7.5){
        m_Drive.driveCartesian(0, 0, 0.33);
        WristAngler.set(0);
      }
      if(m_timer.get() > 7.5 && m_timer.get() < 8.6){
        WristAngler.set(wristController.calculate(encoder.get(), 450));
      }
      if(m_timer.get() > 8.6 && m_timer.get() < 8.8){
        RightClaw.set(Value.kReverse);
        clawClosed = true;
      }
      if(m_timer.get() > 8.8 && m_timer.get() < 11){
        WristAngler.set(wristController.calculate(encoder.get(), 0));
      }
      if(m_timer.get() > 9.4 && m_timer.get() < 10.8){
        m_Drive.driveCartesian(0, 0, 0.33);}
      if(m_timer.get() > 10.8 && m_timer.get() < 14.5){
        m_Drive.driveCartesian(0, 0.33, 0);
      }
      if(m_timer.get() > 12 && m_timer.get() < 14.5){
        WristAngler.set(wristController.calculate(encoder.get(), middleWrist));
        BottomArm.set(Value.kForward);
      }
      if(m_timer.get() > 14.5 && m_timer.get() < 15){
        RightClaw.set(Value.kForward);
        clawClosed = false;
      }
  
        break;


      case placeAndBalance:
      if(m_timer.get() < 0.4){
        UpperArm.set(Value.kForward);
        BottomArm.set(Value.kForward);
        //Set arm to top position
                }
      if(m_timer.get() > 0.6 && m_timer.get() < 3){
        WristAngler.set(wristController.calculate(encoder.get(), 870));
        //set wrist to top position
      }
      if(m_timer.get() > 3 && m_timer.get() < 3.5){
        WristAngler.set(0.0);
      m_Drive.driveCartesian(0, 0.133, 0);
        //stop wrist and drive forward
      }      
      if(m_timer.get() > 3.5 && m_timer.get() < 3.7){
        RightClaw.set(Value.kForward);
        clawClosed = false;
        //open claw to release game piece
      }
      if(m_timer.get() > 3.7 && m_timer.get() < 3.9){
        m_Drive.driveCartesian(0, -0.3, 0);
        //back up
      }
      if(m_timer.get() > 3.6 && m_timer.get() < 4.6){
        WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));}
        //set wrist to bottom position
      if(m_timer.get() > 4.1 && m_timer.get() < 4.6){
        UpperArm.set(Value.kReverse);
        BottomArm.set(Value.kReverse);
        WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
        //set wrist and arm to bottom position
      }
      if(m_timer.get() > 4.6 && m_timer.get() < 5.8 &&(gyro.getYComplementaryAngle() < 8 && gyro.getYComplementaryAngle() > -8)){
        WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
        m_Drive.driveCartesian(0, -0.5, 0);
        //drive backwards(this is where the program stops working)
      }
      if((gyro.getYComplementaryAngle() > 2 || gyro.getYComplementaryAngle() < -2) && m_timer.get() > 6.0){
        WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
        m_Drive.driveCartesian(0, -0.1 * autoLeveler.calculate(gyro.getYComplementaryAngle(), -0.5), 0);
      }
      if(m_timer.get() > 6.4 && gyro.getYComplementaryAngle() < 2 && gyro.getYComplementaryAngle() > -2){
        Brakes.set(Value.kForward);
        WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
      }
      /*if((gyro.getYComplementaryAngle() > 13.9 || gyro.getYComplementaryAngle() < -13.9)){
        balanceTimer = m_timer.get();
m_Drive.driveCartesian(0, -1, 0);
        if(m_timer.get() > balanceTimer + 0.5){
        Brakes.set(Value.kForward);
        WristAngler.set(0.0);
        m_Drive.driveCartesian(0, 0, 0);}
        //stop driving and deploy brakes*/
      
      break;


      case placeAndMobilize:
      if(m_timer.get() < 0.4){
        UpperArm.set(Value.kForward);
        BottomArm.set(Value.kForward);
                }
      if(m_timer.get() > 0.4 && m_timer.get() < 3){
        WristAngler.set(wristController.calculate(encoder.get(), 870) * 0.5);
      }
      if(m_timer.get() > 3 && m_timer.get() < 3.5){
        WristAngler.set(0.0);
      m_Drive.driveCartesian(0, 0.4 / 3.0, 0);  }      
      if(m_timer.get() > 3.5 && m_timer.get() < 3.6){
        RightClaw.set(Value.kForward);
        clawClosed = false;
      }
      if(m_timer.get() > 3.6 && m_timer.get() < 3.8){
        RightClaw.set(Value.kForward);
        m_Drive.driveCartesian(0, -0.3 / 3, 0);
      }
      if(m_timer.get() > 3.8 && m_timer.get() < 3.9){
        m_Drive.driveCartesian(0, 0, 0);
      } 
      if(m_timer.get() > 3.9 && m_timer.get() < 4.9){
        WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));}
      if(m_timer.get() > 4.7 && m_timer.get() < 4.9){
        UpperArm.set(Value.kReverse);
        BottomArm.set(Value.kReverse);
      }
      if(m_timer.get() > 4.9 && m_timer.get() < 8.2){
        WristAngler.set(0.0);
        m_Drive.driveCartesian(0, -0.33, 0);
      }
      if(m_timer.get() > 8.2 && m_timer.get() < 9.6){
        m_Drive.driveCartesian(0, 0, 0.33);
      }

      break;


      case placeAndGrab:
      default:
        // Put default auto code here
        if(m_timer.get() < 0.4){
          UpperArm.set(Value.kForward);
          BottomArm.set(Value.kForward);
                  }
        if(m_timer.get() > 0.4 && m_timer.get() < 3){
          WristAngler.set(wristController.calculate(encoder.get(), 870) * 0.5);
        }
        if(m_timer.get() > 3 && m_timer.get() < 3.5){
          WristAngler.set(0.0);
        m_Drive.driveCartesian(0, 0.4 / 3.0, 0);  }      
        if(m_timer.get() > 3.5 && m_timer.get() < 3.6){
          RightClaw.set(Value.kForward);
          clawClosed = false;
        }
        if(m_timer.get() > 3.6 && m_timer.get() < 3.8){
          RightClaw.set(Value.kForward);
          m_Drive.driveCartesian(0, -0.3 / 3.0, 0);
        }
        if(m_timer.get() > 3.8 && m_timer.get() < 3.9){
          m_Drive.driveCartesian(0, 0, 0);
        } 
        if(m_timer.get() > 3.9 && m_timer.get() < 4.9){
          WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));}
        if(m_timer.get() > 4.7 && m_timer.get() < 4.9){
          UpperArm.set(Value.kReverse);
          BottomArm.set(Value.kReverse);
        }
        if(m_timer.get() > 4.9 && m_timer.get() < 8.2){
          WristAngler.set(0.0);
          m_Drive.driveCartesian(0, -0.33, 0);
        }
        if(m_timer.get() > 8.2 && m_timer.get() < 9.75){
          m_Drive.driveCartesian(0, 0, 0.33);
        }
        if(m_timer.get() > 10.0 && m_timer.get() < 13.0){
          WristAngler.set(wristController.calculate(encoder.get(), pickUpPos) * 0.4);
        }
        if(m_timer.get() > 11.0 && m_timer.get() < 11.6){
          m_Drive.driveCartesian(0, 0.2, 0);
        }
        if(m_timer.get() > 11.6 && m_timer.get() < 11.7){
          m_Drive.driveCartesian(0, 0, 0);
          RightClaw.set(Value.kReverse);
        }
        if(m_timer.get() > 13.1 && m_timer.get() < 15){
          WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
        }
        
        /* 
        if(m_timer.get() > 11.0 && m_timer.get() < 11.5){
          WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
        }
        if(m_timer.get() > 11.5 && m_timer.get() < 12.55){
          m_Drive.driveCartesian(0, 0, 0.33);
        }
        if(m_timer.get() > 12.6 && m_timer.get() < 15.0){
          m_Drive.driveCartesian(0, 0.4, 0);
        }*/
        /* 
        if(m_timer.get() > 9.6 && m_timer.get() < 10){
          WristAngler.set(wristController.calculate(encoder.get(), pickupPOS));
        }
        if(m_timer.get() > 10.5 && m_timer.get() < 10.6){
          RightClaw.set(Value.kReverse);
        }
        if(m_timer.get() > 10.6 && m_timer.get() < 10.8){
          WristAngler.set(wristController.calculate(encoder.get(), 0));
        }
        if(m_timer.get() > 10.8 && m_timer.get() < 12.2){
          WristAngler.set(wristController.calculate(encoder.get(), bottomWrist));
          m_Drive.driveCartesian(0, 0, 1 / 3);}
        if(m_timer.get() > 12.2 && m_timer.get() < 15.1){
          m_Drive.driveCartesian(0, 1 / 3, 0);
        }*/
        break;
  }}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    
    airCompressor.enableDigital();
    compressorOn = true;
    m_timer.reset();
    //encoder.reset();
    wristController.setSetpoint(bottomWrist);
    Brakes.set(Value.kReverse);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
//Teleop code here


if(m_timer.get() > 134){
  Brakes.set(Value.kForward);
  }
    //Set Middle Position
    if(m_operatorStick.getRawButtonPressed(2)){
      BottomArm.set(Value.kForward);
      UpperArm.set(Value.kReverse);
      wristController.setSetpoint(middleWrist);
    }
    //Set Upper Position
    else if(m_operatorStick.getRawButtonPressed(4)){
      BottomArm.set(Value.kForward);
      UpperArm.set(Value.kForward);
      wristController.setSetpoint(UpperWrist);
    }
    //Set Ground Pickup Position
    else if(m_operatorStick.getRawButtonPressed(1)){
      wristController.setSetpoint(pickUpPos);
      BottomArm.set(Value.kReverse);
      UpperArm.set(Value.kReverse);
    }
    //Set Double Player Station Position
    else if(m_operatorStick.getRawButtonPressed(3)){
      wristController.setSetpoint(playStation);
      BottomArm.set(Value.kForward);
      UpperArm.set(Value.kForward);
    }
    else if(m_operatorStick.getPOV()==0){
      wristController.setSetpoint(sliderPickup);
    }
    else if(m_operatorStick.getRawButtonReleased(3) || m_operatorStick.getRawButtonReleased(4) || m_operatorStick.getRawButtonReleased(2) || m_operatorStick.getRawButtonReleased(1) || m_operatorStick.getPOV() == 180){
      wristController.setSetpoint(bottomWrist);
      BottomArm.set(Value.kReverse);
      UpperArm.set(Value.kReverse);}
  


//Open and close claw for Cone or Cube
if(m_operatorStick.getRawButtonPressed(5)){
    if(clawClosed){
      RightClaw.set(Value.kForward);
      clawClosed = false;
    }}
  if(m_operatorStick.getRawButtonPressed(6)){
    RightClaw.set(Value.kReverse);
    clawClosed = true;
  }

//Deploy and retract brakes
if(m_driveStick.getRawButtonPressed(3)){
  Brakes.set(Value.kForward);
}
if(m_driveStick.getRawButton(2)){Brakes.set(Value.kReverse);
}
if(m_operatorStick.getRawButton(9)){
WristAngler.set(m_operatorStick.getRawAxis(1) * 0.5);
}
else if(wristController.getSetpoint() - encoder.get() > 30 || wristController.getSetpoint() - encoder.get() < -30){
  WristAngler.set(wristController.calculate(encoder.get()) * 0.5);
  }
else{
    WristAngler.set(0.00);
  }

//Set the wrist to the setpoint if it isn't already at the setpoint
/*if(wristController.atSetpoint()){
  WristAngler.set(0.0);
}
else if(wristController.atSetpoint() == false){
WristAngler.set(wristController.calculate(encoder.get()) * 0.5);
}*/

//Code for auto aligning with the limelight
if(m_driveStick.getRawButtonPressed(1)){
  limeLightMove = true;
}
if(m_driveStick.getRawButtonReleased(1)){
  limeLightMove = false;
}
if(limeLightMove){
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  double x = tx.getDouble(0.0);
  m_Drive.driveCartesian(x * 0.1, 0, 0);
  }
else if(m_driveStick.getRawButton(5)){
  m_Drive.driveCartesian(m_driveStick.getRawAxis(0), -m_driveStick.getRawAxis(1), m_driveStick.getRawAxis(2), Rotation2d.fromDegrees(gyro.getAngle() + 180));
}
else if(m_driveStick.getRawButton(12)){
  rotateAuto.setSetpoint(gyro.getAngle() + 180);
  m_Drive.driveCartesian(m_driveStick.getRawAxis(0) * 0.75, -m_driveStick.getRawAxis(1) * 0.75, rotateAuto.calculate(gyro.getAngle()), Rotation2d.fromDegrees(gyro.getAngle() + 180));
}
else{
  m_Drive.driveCartesian(m_driveStick.getRawAxis(0) *0.75, -m_driveStick.getRawAxis(1) * 0.75, m_driveStick.getRawAxis(2) * 0.75, Rotation2d.fromDegrees(gyro.getAngle() + 180));
}
  SmartDashboard.putNumber("Encoder", encoder.get());
  SmartDashboard.putBoolean("At Setpoint", wristController.atSetpoint());

if(m_operatorStick.getRawButtonPressed(10)){
  if(compressorOn){
    airCompressor.disable();
    compressorOn = false;
  }
  else if(compressorOn = false){
    airCompressor.enableDigital();
    compressorOn = true;
}
}

SmartDashboard.putNumber("gyro", gyro.getAngle());
// System.out.println(gyro.getAngle());
if(m_driveStick.getRawButton(6)){
  gyro.reset();
}

//Continue Code here  
}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  encoder.reset();
  gyro.reset();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

//WristAngler.set(m_operatorStick.getRawAxis(5) * 0.5);
SmartDashboard.putNumber("Encoder", encoder.get());
if(m_operatorStick.getRawButtonPressed(10)){
  encoder.reset();}
  
SmartDashboard.putNumber("Color Sensor IR", m_colorSensor.getIR());

  if(clawClosed){
    if(m_operatorStick.getRawButtonPressed(5)){
      RightClaw.set(Value.kForward);
      clawClosed = false;
    }}
  else if(!clawClosed){
    if(m_colorSensor.getIR() > 1){
      RightClaw.set(Value.kReverse);
      clawClosed = true;
    }}


if(m_driveStick.getRawButtonPressed(4)){
  autoLevel = true;
}
if(m_driveStick.getRawButtonReleased(4)){
  autoLevel = false;
}
if(autoLevel){
  m_Drive.driveCartesian(0, -0.2 * autoLeveler.calculate(gyro.getYComplementaryAngle(), -0.5), 0);
}


//m_Drive.driveCartesian(m_driveStick.getRawAxis(0) * 0.1, m_driveStick.getRawAxis(1) * 0.1, m_driveStick.getRawAxis(2) * 0.1, Rotation2d.fromDegrees(gyro.getAngle()));}
  
  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
