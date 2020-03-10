/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Spark;

 //**************************************************************************\\
 // The VM is configured to automatically run this class, and to call the    \\
 // functions corresponding to each mode, as described in the TimedRobot     \\
 // documentation. If you change the name of this class or the package after \\
 // creating this project, you must also update the build.gradle file in the \\
 // project.                                                                 \\
 //**************************************************************************\\
public class Robot extends TimedRobot {
  Joystick drive_stick;
  Joystick control_stick;
  DifferentialDrive m_drive;

 //////////////////////////Speed Definitions\\\\\\\\\\\\\\\\\\\\\\\\\

  double m_deadZone;
  double m_driveMotorSpeed;
  double m_driveTurnSpeed;
  double m_intakeSpeed;
  double m_shooterSpeed;
  double ballMotorSpeed;
  double displayCtr;
  double Y;
  double m_Intake; 
  double m_LOutakeLow;
  double m_ROutakeLow;
  double m_LOutakeHigh;
  double m_ROutakeHigh;
  double m_Belt;
  double m_ClimbUp;
  double m_ClimbDown;
  double m_CrawlRight;
  double m_CrawlLeft;
  

 
  static final int IMG_WIDTH = 320;
  static final int IMG_HEIGHT = 240;
  
//////////////////////////BUTTON MAPPINGs\\\\\\\\\\\\\\\\\\\\\\\\\
  
                        // Control Stick \\
  static final int BTNBELT = 1;        //Belt Forward
  static final int BTNINTAKE = 2;        //Intake
  static final int BTNSHOOTERHIGH = 3;   //Shooter High
  static final int BTNSHOOTERLOW = 4;    //Shooter Low
  static final int BTNBELTREVERSE = 5;   // Belt Backward
  static final int BTNCLIMBUP = 6;       //Hook Extent
  static final int BTNCLIMBDOWN = 7;     //Hook Retract
  static final int BTNCRAWLLEFT = 8;     // Winch CounterClockwise
  static final int BTNCRAWLRIGHT = 9;    // Winch Clockwise
  static final int BTNINTAKEBACK = 10;   // Intake Reverse
  static final int BTNSHOOTERBACK = 11;  // Reverse Shooter
  
                         // Drive Stick \\
  static final int BTNSTOPALL = 7;       // Stops all motors besides drive motors
  static final int BTNColorRed = 8;      // Red Color Spinner 
  static final int BTNColorGreen = 9;    // Green Color Spinner
  static final int BTNColorBlue = 10;    // Blue Color Spinner
  static final int BTNColorYellow = 11;  // Yellow Color Spinner

  //////////////////////////Color Count definitions\\\\\\\\\\\\\\\\\\\\\\\\\

  static final int NONE = -1;
  static final int greenCount = 1;
  static final int redCount = 1;
  static final int yellowCount = 1;
  static final int blueCount = 1;
  //temporary variables for colors and rotations
  static int total = 0;
  static int rotation = 0;
  static int greenTemp= 0;
  static int redTemp = 0;
  static int blueTemp = 0;
  static int yellowTemp = 0;
  //startTotal is used to reset any variable
  static final int startTotal = 0;
  //dont touch anything with endTotal it works somehow
  static final int endtotal = 6;
  //used to increment rotations because ++ was not working at the time of test
  static final int rotationAdd = 1;

  //////////////////////////Motor Set-up\\\\\\\\\\\\\\\\\\\\\\\\\
 
  PWMVictorSPX m_frontRight = new PWMVictorSPX(6);
  PWMVictorSPX m_frontLeft = new PWMVictorSPX(7);
  PWMVictorSPX m_rearRight = new PWMVictorSPX(8);
  PWMVictorSPX m_rearLeft = new PWMVictorSPX(9);

  WPI_VictorSPX m_frontLight = new WPI_VictorSPX(10);
 //////////////////////////Speed Control Group for driving\\\\\\\\\\\\\\\\\\\\\\\\\
  SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
  SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
  
 CANSparkMax m_IntakeMotor = new CANSparkMax(8, MotorType.kBrushless);
 Talon m_ROutakeMotor = new Talon(0); 
 Talon m_LOutakeMotor = new Talon(1);
 Talon m_BeltMotor = new Talon(2);
 Talon m_ClimbMotor = new Talon(3);
 Talon m_ColorSpinner  = new  Talon(4);
 Spark CrawlMotor = new Spark(5);
  int retries;
  
  Timer timer = new Timer();
  static boolean stage1 = false;
  static boolean stage2 = false;
  static boolean stage3 = false;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  
  //****************************************************************************\\
  // This function is run when the robot is first started up and should be used \\
  // for any initialization code.                                               \\
  //****************************************************************************\\
  
  
  @Override
  public void robotInit() {

    System.out.println("Robot Init: ");

    //////////////////////////Variable assignment\\\\\\\\\\\\\\\\\\\\\\\\\

    m_deadZone = 0.1;
    m_driveMotorSpeed = 1;
    m_driveTurnSpeed = 0.75;
    Y = 0; //don't delete the sacred Y
    //Intake and Outake motor speeds
    m_Intake = 0.30;
    m_ROutakeLow = -0.3;
    m_LOutakeLow = 0.3;
    m_ROutakeHigh = -.7;
    m_LOutakeHigh = .7;
    m_Belt = .9;
    m_ClimbUp = .7;
    m_ClimbDown = -.5;
    m_CrawlRight = .5;
    m_CrawlLeft = -.5;
    m_zero = 0.0;

    drive_stick = new Joystick(0);
    control_stick = new Joystick(1);

    m_drive = new DifferentialDrive(m_left, m_right);
    m_drive.setExpiration(0.50);
    m_drive.arcadeDrive(0, 0, true);
    m_drive.setSafetyEnabled(false);

    System.out.println("END Robot Init: ");

    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);    
  }

  //*********************************************************************************\\
  // This function is called every robot packet, no matter the mode. Use this for    \\
  // items like diagnostics that you want ran during disabled, autonomous,           \\
  // teleoperated and test.                                                          \\
  //                                                                                 \\                                                                            \\
  // This runs after the mode specific periodic functions, but before LiveWindow     \\
  // and SmartDashboard integrated updating.                                         \\
  //*********************************************************************************\\
  

  @Override
  public void autonomousInit() {
    System.out.println("StartAutoInit");
    timer.start();
    //retries = 50; // wait up to 2 secs for game data
    System.out.println("EndAutoInit");
  }

  //*****************************************************************************\\
  // This function is called periodically during autonomous.                     \\
  //Using timer we have a different function for every interval                  \\
  //from 0-2 we set the drive motors to .15 and -.15 to drive                    \\
  //from 2-5 we turn right right motor is set to .15 to drive the turn           \\
  //from 5-9 we turn left by putting power to left motor                         \\
  //for the rest of autonomous we run the belt and shooter                       \\
  //have to be calibrated to the right distancees                                \\
  //have to have code for every case                                             \\
  //look up how to use java switch cases look at the code commented out line 277 \\
  //Keller is good at math he knows how to calculate distances and speeds        \\
  //you can change the time intervals and anything in the code below             \\
  //you must use while loops because you want to run the function while          \\
  //it is in that time interval.                                                 \\
  //*****************************************************************************\\
  @Override
  public void autonomousPeriodic() {
    gyro.reset();
    int roboLocation = DriverStation.getInstance().getLocation();//gets driver station location
    //test to see how long to 7 feet
    while(timer.get()<4){
      m_left.set(-0.3);
      m_right.set(0.3);
      m_LOuttakeMotor.set(0.5);
      m_ROuttakeMotor.set(0.5);
    }
    //INTERVAL 1
    
    while(timer.get()<4){
      m_BeltMotor.set(0.90);
      m_LOutakeMotor.set(m_LOutakeHigh);
      m_ROutakeMotor.set(m_ROutakeHigh);
      
    }
    //INTEVAL2
    while(timer.get()>=4 && timer.get()<8){
      m_BeltMotor.set(0);
      m_LOutakeMotor.set(m_zero);
      m_ROutakeMotor.set(m_zero);
      m_left.set(m_zero);
      m_right.set(0.25);
    }
    while(timer.get()>=8 && timer.get()<9){
      m_left.set(0.15);
      m_right.set(-0.15);
    }
    while(timer.get()>=9&&timer.get()<11){
      m_left.set(m_zero);
      m_right.set(0.25);
    }
    while(timer.get()<15){
      m_right.set(m_zero);
      m_left.set(m_zero);
      
    }
    m_LOutakeMotor.set(m_zero);
    m_ROutakeMotor.set(m_zero);
    m_BeltMotor.set(m_zero);
    


    /*
    switch(roboLocation){
      case 1:
      //code for the robot starting on the left
      
      break;
      case 2:
      //code for the robot startinf in the middle
      break;
      case 3:
      //code for the robot starting on the right
      break;
  
    }
    */
    }

  //********************************************************\\
  // This function is called at the start of Teloeop.       \\
  //********************************************************\\
  @Override
  public void teleopInit() {
    m_drive.arcadeDrive(0.0,0.0);
    //System.out.println("TeleOpInit");
    //updateDisplays();
  }

  //*******************************************************************\\
  // This function is called periodically during operator control.     \\
  //*******************************************************************\\
  @Override
  public void teleopPeriodic() {

    // Get Drive Joystick input for arcade driving

    double X = getJoystickValue(drive_stick, 1) * m_driveMotorSpeed;
    double Z = getJoystickValue(drive_stick, 2) * m_driveTurnSpeed;


    
    if (lastStage > 1){
      X = X *0.75; //reduce to 75% speed if lift is up beyond level 1
    }
    m_drive.arcadeDrive(-X, Z, true); // Drive the robot
    
    //////////////////////////Main Methods for Operation\\\\\\\\\\\\\\\\\\\\\\\\\
   
    ballControl(); // call ball control routine(for intake and shooting)
    ballControlSensor();
    beltControl();
    Climber();
    BarCrawl();
    StopAll();
  }

  @Override
  public void robotPeriodic() {
    //**************************************************************************************\\
    // The method GetColor() returns a normalized color value from the sensor and can be    \\
    // useful if outputting the color to an RGB LED or similar. To                          \\
    // read the raw color, use GetRawColor().                                               \\
    //                                                                                      \\
    // The color sensor works best when within a few inches from an object in               \\
    // well lit conditions (the built in LED is a big help here!). The farther              \\
    // an object is the more light from the surroundings will bleed into the                \\
    // measurements and make it difficult to accurately determine its color.                \\
    //**************************************************************************************\\
    Color detectedColor = m_colorSensor.getColor();

    //*****************************************************\\
    // Run the color match algorithm on our detected color \\
    //*****************************************************\\
        String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
  
    //*******************************************************************************\\
    // Open Smart Dashboard or Shuffleboard to see the color detected by the sensor. \\
    //*******************************************************************************\\
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
  

  @Override
  public void testPeriodic() {
  }

  //*******************************************************************\\
  //This function is used to read joystick & eliminate deadzone issues \\
  //*******************************************************************\\

  public double getJoystickValue(Joystick joystick, int iKey) {
    double dVal = joystick.getRawAxis(iKey);
    if (Math.abs(dVal) < m_deadZone) {
      return 0;
    } else {
      return dVal;
    }
  }

  //*******************************************\\
  // This function controls the shooter system \\
  //*******************************************\\  
  public void ballControl() {

    if (control_stick.getRawButtonPressed(BTNINTAKE) == true) {
      m_IntakeMotor.set(m_Intake);
    }
    if (control_stick.getRawButtonReleased(BTNINTAKE) == true) {
      m_IntakeMotor.set(m_zero);

    }
    if (control_stick.getRawButtonPressed(BTNINTAKEBACK) == true) {
      m_IntakeMotor.set(-m_Intake);
    }
    if (control_stick.getRawButtonReleased(BTNINTAKEBACK) == true) {
      m_IntakeMotor.set(m_zero);

    }
    if (control_stick.getRawButtonPressed(BTNSHOOTERHIGH) == true) {
      m_LOutakeMotor.set(m_LOutakeHigh); //This section defines the high speed shot
      m_ROutakeMotor.set(m_ROutakeHigh);
    }
    if (control_stick.getRawButtonReleased(BTNSHOOTERHIGH) == true) {
      m_LOutakeMotor.set(m_zero);
      m_ROutakeMotor.set(m_zero);
    }
    if (control_stick.getRawButtonPressed(BTNSHOOTERLOW) == true) {
      m_LOutakeMotor.set(m_LOutakeLow); //This section defines the low speed shot
      m_ROutakeMotor.set(m_ROutakeLow);
    }
    if (control_stick.getRawButtonReleased(BTNSHOOTERLOW) == true) {
      m_LOutakeMotor.set(m_zero);
      m_ROutakeMotor.set(m_zero);
    }
    if (control_stick.getRawButtonPressed(BTNSHOOTERBACK) == true) {
      m_LOutakeMotor.set(-.70);
      m_ROutakeMotor.set(.70);
    }
    if (control_stick.getRawButtonReleased(BTNSHOOTERBACK) == true) {
      m_LOutakeMotor.set(m_zero);
      m_ROutakeMotor.set(m_zero);
    }
  }
 //**************************************************\\
 //This function controls they hook extending system \\
 //**************************************************\\
  public void Climber() {
    if (control_stick.getRawButtonPressed(BTNCLIMBUP) == true){
      m_ClimbMotor.set(m_ClimbUp);
    }
    if (control_stick.getRawButtonReleased(BTNCLIMBUP) == true){
      m_ClimbMotor.set(m_zero);
    }
    if (control_stick.getRawButtonPressed(BTNCLIMBDOWN) == true) {
      m_ClimbMotor.set(m_ClimbDown);
    }
    if (control_stick.getRawButtonReleased(BTNCLIMBDOWN) == true) {
      m_ClimbMotor.set(m_zero);
    }
  }
 //******************************************\\
 //This function controls they winch system  \\
 //******************************************\\
public void BarCrawl() {
  if (control_stick.getRawButtonPressed(BTNCRAWLRIGHT) == true){
    CrawlMotor.set(m_CrawlRight);
  }
  if (control_stick.getRawButtonReleased(BTNCRAWLRIGHT) == true){
    CrawlMotor.set(m_zero);
  }
  if (control_stick.getRawButtonPressed(BTNCRAWLLEFT) == true) {
    CrawlMotor.set(m_CrawlLeft);
  }
  if (control_stick.getRawButtonReleased(BTNCRAWLLEFT) == true) {
    CrawlMotor.set(m_zero);
  }
}
 //******************************************\\
 //This function controls they belt system \\
 //******************************************\\
  public void beltControl() {

     if (control_stick.getRawButtonPressed(BTNBELT) == true) {
     m_BeltMotor.set(m_Belt); 
     }
     if (control_stick.getRawButtonReleased(BTNBELT) == true) {
      m_BeltMotor.set(m_zero); 
    }
    if(control_stick.getRawButtonPressed(BTNBELTREVERSE)==true){
      m_BeltMotor.set(-m_Belt);
    }
    if(control_stick.getRawButtonReleased(BTNBELTREVERSE)==true){
      m_BeltMotor.set(m_zero);
    }
  }
  //***************************************************************\\
  // This function stops all motors from running if issue occurs   \\
  //***************************************************************\\
  
  public void StopAll() {
    if (drive_stick.getRawButton(BTNSTOPALL) = true) {
      m_LOutakeMotor.set(m_zero);
      m_ROutakeMotor.set(m_zero);
      m_BeltMotor.set(m_zero);
      m_ColorSpinner.set(m_zero);
      m_ClimbMotor.set(m_zero);
      CrawlMotor.set(m_zero);
    }
  }
  
  //*************************************************************************\\
  // This function runs motor for 3 rotations and stops on specified color   \\
  //*************************************************************************\\
  public int getColorCmd() {
      if (drive_stick.getRawButton(BTNColorRed) == true)
        return 0;
      if (drive_stick.getRawButton(BTNColorGreen) == true)
        return 1;
      if (drive_stick.getRawButton(BTNColorBlue) == true)
        return 2;
      if (drive_stick.getRawButton(BTNColorYellow) == true)
       return 3;
      return NONE;
    }
  
  public void ballControlSensor() {
  //gets the color from the sensor and then matches it with its "most likely" color
  //use detected.color for comparisons because that's what is needed to compare colors
  Color match = m_colorSensor.getColor();
  ColorMatchResult detected = m_colorMatcher.matchClosestColor(match);
  int button = getColorCmd();
//while button 0 is held the color sensor will do the following lines of code
  if (button>=0){
    //following if statements compare the detected colors to blue,green,red,yellow
    //if color is detected the temporary variable is set to 1 
    if(detected.color == kBlueTarget){
      blueTemp = blueCount;
      m_ColorSpinner.set(.6);
    }
    if(detected.color== kGreenTarget){
        greenTemp = greenCount;
        m_ColorSpinner.set(.6);
    }  
    if(detected.color == kRedTarget){
        redTemp = redCount;
        m_ColorSpinner.set(.6);
    } 
    //only made the temporary variable 1 if the confidnce was over .95 because white was getting identified as yellow with low confidence

    if(detected.color== kYellowTarget && detected.confidence>0.95 ){
        yellowTemp = yellowCount;
        m_ColorSpinner.set(.6);
      }
    // creates a total of temporary variables
    int total = yellowTemp+redTemp+blueTemp+greenTemp;
    //if the total is 4 that means we ran through all the colors and we reset the temporary variables to 0
    //and add 1 to the rotation count
      if(total==4){
        rotation += rotationAdd;
        total = startTotal;
        yellowTemp = startTotal;
        redTemp= startTotal;
        blueTemp= startTotal;
        greenTemp = startTotal;
        
      }
      //if the rotation surpasses 3 and the color matches with the color we want to stop at
      //the motor speed is set to 0 to stop running
  if(rotation>6 && detected.color == kRedTarget && button==0){
      m_ColorSpinner.set(m_zero);
      total = endtotal;
        }

  if(rotation>6 && detected.color == kGreenTarget && button==1){
    m_ColorSpinner.set(m_zero);
    total = endtotal;
    }
  if(rotation>6 && detected.color == kBlueTarget && button==2){
    m_ColorSpinner.set(m_zero);
    total = endtotal;
      }
  if(rotation>6 && detected.color == kYellowTarget && detected.confidence>0.95 && button==3){
    m_ColorSpinner.set(m_zero);
    total = endtotal;
        }
  }
  //if button 0 is not held anymore it resets the rotations to 0
  if (button<0){
    rotation = startTotal;
    }
 
  //End all
