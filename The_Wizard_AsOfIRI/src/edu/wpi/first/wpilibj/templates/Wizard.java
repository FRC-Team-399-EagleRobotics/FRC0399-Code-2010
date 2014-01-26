
package edu.wpi.first.wpilibj.templates;

import com.sun.squawk.util.MathUtils;
import edu.wpi.first.addons.CANJaguar;
import edu.wpi.first.addons.CANJaguar.ControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Dashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.addons.DSInputs;

public class Wizard extends IterativeRobot {
    // CAN Arbitration IDs start at 2, DO NOT USE 1 
    final int LeftDriveCANID     = 5;
    final int RightDriveCANID    = 2;
    final int Kicker1CANID       = 3;
    final int FrontRollerCANID   = 4;
    final int leftJoystick    = 1;
    final int rightJoystick   = 2;
    final int operatorGamepad = 3;
    final int HighGearSolenoidID = 1;
    final int LowGearSolenoidID  = 2;
    final short LeftCANEncoderCPR = 250;
    final double LeftCANFaultTime  = 1.0; // One second fault time
    final short RightCANEncoderCPR = 250;
    final double RightCANFaultTime  = 1.0;
    SpeedController LeftDr; // Usually instance of CANJaguar
    SpeedController RightDr;
    RobotDrive drive;
    SpeedController Kicker1;
    SpeedController Roller;
    Joystick left     = new Joystick(leftJoystick);
    Joystick right    = new Joystick(rightJoystick);
    Joystick operator = new Joystick(operatorGamepad);
    DriverStation ds = DriverStation.getInstance();
    Compressor compressor = new Compressor(10,2);
    //Ultrasonic leftUS  = new Ultrasonic(11,12);
    //Ultrasonic rightUS = new Ultrasonic(13,14);
    DigitalInput magSwitch    = new DigitalInput(9);
    DigitalInput trampolLimit = new DigitalInput(1);
    Solenoid trampol1   = new Solenoid(1);
    Solenoid trampol2   = new Solenoid(2);
    Solenoid shift1     = new Solenoid(7);
    Solenoid shift2     = new Solenoid(8);
    DriverStationEnhancedIO IO = DriverStation.getInstance().getEnhancedIO();
    DriverStationLCD LCD       = DriverStationLCD.getInstance();
    Relay coldCathode = new Relay(1);
    Servo horiz = new Servo(3);
    Servo vert  = new Servo(2);
    boolean shifted = false;
    boolean gear = false;
    boolean trampolExt = false;
    boolean trampol    = false;
    boolean servEnable = true;
    boolean oneShot    = false;
    Encoder kicker = new Encoder(11,12);
    ADXL345DigitalAccelerometer accel = new ADXL345DigitalAccelerometer(4);
    double KnobIn       = 0.0;

/*
    final double YP = -0.005625; //0.045; // 0.0068; // Yaw heading proportional
    final double YI = 0.0; //0.000044;   // 0.000225; // Yaw heading integral
    final double YD = 0.0; //0.0009;  // 0.08;  // Yaw heading derivative

    double YawIntegral = 0.0;
    double YawPrevError = 0.0;
    double HeadingTolerance = 10; // 10 degrees tolerance

    final double DP = 0.05; //0.05;// 0.0038; // Distance proportional
    final double DI = 0.00001; //0.00001; //0.000063; // Distance integral
    final double DD = 0.0; //0.003;  //0.03125; // Distance derivative

    double DistanceIntegral = 0.0;
    double DistancePrevError = 0.0;
    double DistanceTolerance = 4; // 3 inch tolerance

    Gyro yaw;
    double distancePerRevolution = (6*Math.PI) / (2.46*12.0);
    double X = 16.833, Y = 0, theta = 270;

    double[][] points =
    {
        {13,6.0},
        {9, 4.5},
        {8.75, 4.5},
        {8,6.0},
        {6, 4.5},
    };
*/
    public void disabledInit()
    {
    }
    public void disabledPeriodic()
    {
        //Y= 1.16666 + 3.0;
        System.out.println("K En " + kicker.getRaw());
        System.out.println("Mag Switch " + !magSwitch.get());

        if(!magSwitch.get()){
            kicker.reset();
        }


        int field = fieldSelect();

        lcdPrint(1);

        kick = false;
    }

    Gyro yaw;

    public void robotInit() 
    {
        yaw = new Gyro(1);

        kicker.start();

        accel.intitialize();
        accel.setRange(accel.DATA_FORMAT_16G);

        LeftDr = createCANJaguar(LeftDriveCANID, ControlMode.kPercentVoltage);
        RightDr = createCANJaguar(RightDriveCANID, ControlMode.kPercentVoltage);
        Kicker1 = createCANJaguar(Kicker1CANID, ControlMode.kPercentVoltage);
        Roller = createCANJaguar(FrontRollerCANID, ControlMode.kPercentVoltage);

        compressor.start();

        //0.5 seconds
        ((CANJaguar)LeftDr).configFaultTime(0.5);
        ((CANJaguar)RightDr).configFaultTime(0.5);

        ((CANJaguar)LeftDr).setPositionReference(CANJaguar.PositionReference.kEncoder);
        ((CANJaguar)LeftDr).configEncoderCodesPerRev((short)250);
        ((CANJaguar)RightDr).setPositionReference(CANJaguar.PositionReference.kEncoder);
        ((CANJaguar)RightDr).configEncoderCodesPerRev((short)250);

        drive = new RobotDrive(LeftDr,RightDr);
                Watchdog.getInstance().setEnabled(false);
    }
    boolean calibrated = false;
    /**
     * This function is called periodically during autonomous
     */

    //int point = 0;

    final double AngleP = -0.005625;
    final double AngleI = 0.0;
    final double AngleD = 0.0;

    double AngleIntegral = 0.0;
    double AngleDerivative = 0.0;

    double angleToDrive = 0.0;
    
    long startTime;

    void driveToAngle(double speed, double angle)
    {
        double normalizedAngle = yaw.getAngle();
        normalizedAngle /= 360.0;
        normalizedAngle -= (int) normalizedAngle;
        normalizedAngle *= 360.0;
        normalizedAngle += 180;
        double angleError = (angle - normalizedAngle);
        AngleIntegral += angleError;
        double turn = (AngleP * angleError) + (AngleI * AngleIntegral) + (AngleD * (angleError - AngleDerivative));
        AngleDerivative = angleError;
        drive.tankDrive(speed+turn, speed-turn);
        
    }

    public void autonomousInit()
    {
        
        startTime = System.currentTimeMillis();
        kick = false;
        setPoint = -90;
    }

    public void autonomousPeriodic()
    {

       /*Change the appropriate angle to
        * angleToDrive
        */

        angleToDrive = 0.0;

        final int field = fieldSelect();
        if(field == 1) {
            angleToDrive = 0.0;  //change for NEAR FIELD
        } else if(field == 2) {
            angleToDrive = 0.0; //Change for MID FIELD
        } else if(field == 3) {
            angleToDrive = 0.0; //Change for FAR FIELD
        }

       if(!magSwitch.get()){
            kicker.reset();
       }

       long elapsed = System.currentTimeMillis() - startTime; 
       if(elapsed < 1625)
       {
            rollers(.75);
            driveToAngle(-0.8, 180);
       }else if(elapsed < 2875){ // delta d = 1250
           driveToAngle(0.8, 160);
       }else if(elapsed < 3100){  // delta d = 250
           driveToAngle(0.0, 160);
       }else if(elapsed < 4126)
       {   if(!kick)
           {
           Kicker1.set(0.0);
           kick = true;
           }
       }else if(elapsed < 4426)
        {
           if(kick)
           {
           kick = false;
           }
        }else if (elapsed < 4926)
       {
            if(!kick)
            {
           setPoint = -90;
            }
       }
      /* }else if(elapsed < 3175){ // delta d = 250
           driveToAngle(0.0, 180);
       }else if(elapsed < 3875){ // delta d = 500
           driveToAngle(-0.8, 180);
       }else if(elapsed < 4000){
           driveToAngle(0.0, 180);
       }*/


       if(!kick)
         {
            P = -0.008;
            I = -0.00006;
            D = -0.000000001;
            double error = (setPoint - kicker.get());
            Integral += error;
            double out = P*error + I*Integral + D*(error - prevErr);
            prevErr = error;
            Kicker1.set(out);
            System.out.println("Position set");
        }else{

            if(kicker.get() < 50){
		Kicker1.set(-1);
            }else{
                kick = false;
                Kicker1.set(0.0);
                //Integral = 0.0;
                System.out.println("FALSE ");
                setPoint = -90; // Magnet setpoint
            }
            System.out.println("ENCODER " + kicker.get());
		System.out.println("KICK!!!");
	}


     /**
       updatePosition();
       double[] dest = null;
       if(point < points.length)
       {
           dest = points[point];
       } else
       {
           drive.tankDrive(0.0, 0.0);
           return;
       }
       double dY = dest[1], dX = dest[0];

       double xDist = MathUtils.pow(dX - X, 2.0);
       double yDist = MathUtils.pow(dY - Y, 2.0);
       double Distance_Set = Math.sqrt(xDist + yDist);
       double Dist_error = Distance_Set;

       // if within tolerance on point, go to the next one
       if(Math.abs(dX - X) < DistanceTolerance
                    && Math.abs(dY - Y) < DistanceTolerance )
       {
           DistanceIntegral = 0;
           DistancePrevError = 0;
           YawIntegral = 0;
           //YawPrevError = 0;
           drive.tankDrive(0, 0);
           point++;
           return;
       }

       //Get an angle to the desired point using arc tangent and convert to deg.
       double Yaw_Set = (180*MathUtils.atan2(dY-Y, dX - X))/Math.PI;

       double Yaw_error = Yaw_Set - theta;

       if (Math.abs(Yaw_error) > ((360 - 0) / 2))//Correct for 359.9-0wraparound
       {
         if (Yaw_error > 0)
         {
           Yaw_error = Yaw_error - 360 + 0;
         } else {
           Yaw_error = Yaw_error + 360 - 0;
         }
       }
       
       YawIntegral += Yaw_error;
       double turn = (YP * Yaw_error)
                        + (YI * YawIntegral)
                            + (YD * (Yaw_error - YawPrevError));
       YawPrevError = Yaw_error;


       DistanceIntegral += Dist_error;
       double speed = (DP * Dist_error)
                        + (DI * DistanceIntegral)
                            + (DD * (Dist_error - DistancePrevError));
       DistancePrevError = Dist_error;

       if(speed > .8){ speed = .8;}

       drive.tankDrive((speed + turn), speed - turn);
      * */
       updateDashboard();
    }
    double P = 0.0, I = 0.0, D = 0.0;
    double Integral = 0.0;
    double prevErr  = 0.0;
    double setPoint = 0.0;
    boolean levelButton = false;
    boolean levelSetPoint = false;
    boolean kick = false;
    public void teleopInit()
    {
        kick = false;
        horiz.set(0.5);
        vert.set(1.0);
        setPoint = 0.0;
        Watchdog.getInstance().setEnabled(false);
    }
    public void teleopPeriodic()
    {
        AxisCamera.getInstance();

        shifters();
        drive();
        rollers(-(operator.getY())*0.75);
        deflectors();
        kicker();
        //updatePosition();
        updateDashboard();
    }

    public void drive()
    {
         try {
            if(IO.getDigital(9)) {
                if(IO.getDigital(10))
                    drive.arcadeDrive(-left.getY(), -left.getX());
                else
                    drive.arcadeDrive(left.getY(), left.getX());
                IO.setLED(1, true);
            } else {
                IO.setLED(1, false);
                if(IO.getDigital(10))
                    tankDrive(-left.getY(), -right.getY());
                else
                    tankDrive(left.getY(), right.getY());
                
            }
        } catch(DriverStationEnhancedIO.EnhancedIOException ex) {
            ex.printStackTrace();
            drive.arcadeDrive(left.getY(), left.getX());
        }
    }
    public void rollers(double speed)
    {
        Roller.set(speed);
    }
    public void kicker()
    {
        if(!magSwitch.get()){
            kicker.reset();
        }

	if((operator.getRawButton(1) || operator.getRawButton(2) || operator.getRawButton(3) || operator.getRawButton(4)) && !levelSetPoint)
	{
		if(operator.getRawButton(1)){
			setPoint = 0;
                }else if(operator.getRawButton(2)){
			setPoint = -45;
                }else if(operator.getRawButton(3)){
			setPoint = -90;
                }else if(operator.getRawButton(4)){
			setPoint = -95;
                }
		levelSetPoint = true;
	}else if(!(operator.getRawButton(1) || operator.getRawButton(2) || operator.getRawButton(3) || operator.getRawButton(4)))
	{
		levelSetPoint = false;
	}

        if(operator.getRawButton(6) && !kick)
        {
            Kicker1.set(0.0);
            kick = true;
        }

        if(!kick)
        {
            P = -0.008;
            I = -0.00006;
            D = -0.000000001;
            double error = (setPoint - kicker.get());
            Integral += error;
            double out = P*error + I*Integral + D*(error - prevErr);
            prevErr = error;
            Kicker1.set(out);
        }else{

            if(kicker.get() < 50){
		Kicker1.set(-1);
            }else{
                kick = false;
                Kicker1.set(-0.0);
                Integral = 0.0;
                setPoint = 0; // Magnet setpoint
            }
		System.out.println("KICK!!!");
	}
    }
    public void deflectors()
    {
        if(operator.getRawButton(8) && !trampolExt)
        {
            trampol = !trampol;
            if(trampol)
            {
                horiz.set(0.5);
                vert.set(1.0);
            }else{
                horiz.set(0.5);
                vert.set(0.5);
            }
	    setPoint = 0;
            kick = false;
            trampol1.set(trampol);
            trampol2.set(trampol);
            trampolExt = true;
        } else if(!operator.getRawButton(8))
        {
           trampolExt = false;
        }
    }
    public void shifters()
    {
        if(left.getTrigger() && !shifted)
        {
            gear = !gear;
            shift1.set(gear);
            shift2.set(!gear);
            shifted = true;
        } else if(!left.getTrigger())
        {
            shifted = false;
        }
    }
    public void tankDrive(double leftV, double rightV)
    {
        try{
           drive.tankDrive(leftV, rightV);
        }catch(Throwable t){
           if(LeftDr instanceof CANJaguar)
           {
               LeftDr = null;
               LeftDr = createCANJaguar(LeftDriveCANID, ControlMode.kPercentVoltage);
           }
           if(RightDr instanceof CANJaguar)
           {
               RightDr = null;
               RightDr = createCANJaguar(RightDriveCANID, ControlMode.kPercentVoltage);
           }
           System.out.println(t.getMessage());
       }
    }
    CANJaguar createCANJaguar(int ID, ControlMode mode)
    {
        try{
            return new CANJaguar(ID, mode);
        }catch(Throwable t){
            System.out.println("CAN Bus failure to init Jag # " + ID + "\n"
                                + t.getMessage());
            return null;
        }
    }

    double Left_Prev_Dist = 0;
    double Right_Prev_Dist = 0;
/*
    public void updatePosition()
    {
       theta = yaw.getAngle();
       theta /= 360.0;
       theta -= (int) theta;
       theta *= 360.0;
       double leftd = ((CANJaguar)LeftDr).getPosition() * distancePerRevolution;
       double rightd = ((CANJaguar)RightDr).getPosition() * -distancePerRevolution;
        System.out.println("Left " + leftd + " Right " + rightd);
       double Dleft = (leftd - Left_Prev_Dist);
       double Dright = (rightd - Right_Prev_Dist);
       double forwardDeltaD = ( Dleft + Dright)/2.0;
       Left_Prev_Dist = leftd;
       Right_Prev_Dist = rightd;
	X += forwardDeltaD*Math.cos((Math.PI * theta)/180.0);
	Y += forwardDeltaD*Math.sin((Math.PI * theta)/180.0);
        System.out.println("X : " + X + " Y : " + Y);
    }
*/

     /**
       Autonomous is HIGH priority
        Left Percent
        Right Percent
        Kicker Strength percent * 100
        Target Acquired boolean
        Ball Loaded ball loaded boolean
        Zone int 0 - 3
        Ball 1 X
        Ball 1 Y
        Ball 2 X
        Ball 2 Y
        Ball 3 X
        Ball 3 Y
        Robot X
        Robot Y

       Teleoperated: LOW Priority
        High Gear    boolean
        Target Acquired boolean
        Ball loaded  boolean
        Kicker Strength % * 100
        Left Amp
        Right Amp
        PSI
        Total current
        Left drive OK? bool
        Right drive OK? bool
        Roller bool
        kicker bool
        */

    long blinkStart = System.currentTimeMillis();

    public int fieldSelect() {

        try {
            int field;
            KnobIn = IO.getAnalogIn(2);
            if(KnobIn >= 1.0 && KnobIn <= 1.9){
                field = 1;
            } else if(KnobIn >= 2.0 && KnobIn <= 2.9) {
                field = 2;
            } else if(KnobIn >= 3.0) {
                field = 3;
            } else {
                field = 0;
            }
                return field;
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
                System.out.println(ex.getMessage());
                return -1;
        }

    }

    public boolean switchGet(int switchReq) {

        try {
            switchReq += 8;
                if(switchReq < 9 || switchReq > 12){
                    return false;
                }
            boolean switchReturned = IO.getDigital(switchReq);
            return switchReturned;
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
            System.out.println(ex.getMessage());
            return false;
        }

    }

    public int autonMode() {
        int sw1;
        int sw2;
        int sw3;
        int sw4;
        int mode;

        if(switchGet(1)) {
            sw1 = 1;
        } else {
            sw1 = 0;
        }
        if(switchGet(2)) {
            sw2 = 2;
        } else {
            sw2 = 0;
        }
        if(switchGet(3)) {
            sw3 = 4;
        } else {
            sw3 = 0;
        }
        if(switchGet(4)) {
            sw4 = 8;
        } else {
            sw4 = 0;
        }
        
        mode = (sw1 + sw2 + sw3 + sw4);
        
        return mode;
    }
    
    public void lcdPrint(int sel) {
        if(sel == 1) {
        
            int field = fieldSelect();

            if(field == 1) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, ">>>>>>>>>>>>>>>>>>>>");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "^    NEAR FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "^    NEAR FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "<<<<<<<<<<<<<<<<<<<<");
            } else if(field == 2) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, ">>>>>>>>>>>>>>>>>>>>");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "^    MID  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "^    MID  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "<<<<<<<<<<<<<<<<<<<<");
            } else if(field == 3) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, ">>>>>>>>>>>>>>>>>>>>");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "^    FAR  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "^    FAR  FIELD    ^");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "<<<<<<<<<<<<<<<<<<<<");
            } else {
                LCD.println(DriverStationLCD.Line.kUser2, 1, "!!!!!!!!!!!!!!!!!!!!");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "!       ERROR      !");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "!  IO BOARD ERROR  !");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "!!!!!!!!!!!!!!!!!!!!");
            }
               
        } 
        else if(sel == 2) {
                LCD.println(DriverStationLCD.Line.kUser2, 1, "                    ");
                LCD.println(DriverStationLCD.Line.kUser3, 1, "                    ");
                LCD.println(DriverStationLCD.Line.kUser4, 1, "                    ");
                LCD.println(DriverStationLCD.Line.kUser5, 1, "                    ");
        }
        
        LCD.updateLCD();
        
    }
    

    public void updateDashboard()
    {

       Dashboard auto = ds.getDashboardPackerHigh();
       Dashboard tele = ds.getDashboardPackerLow();

       CANJaguar leftd = (CANJaguar) LeftDr;
       CANJaguar rightd = (CANJaguar) RightDr;
       CANJaguar roll  = (CANJaguar) Roller;
       CANJaguar kicke = (CANJaguar) Kicker1;

       double totalCurrent =leftd.getOutputCurrent() + rightd.getOutputCurrent()
                           +roll.getOutputCurrent()  + kicke.getOutputCurrent();

       double kickerPower = 22.5722 -(0.7959 * kicker.get());
       double PSI = 0.0;
       if(ds.isAutonomous())
       {
            auto.addDouble((leftd.getBusVoltage() != 0.0)?leftd.getOutputVoltage()/leftd.getBusVoltage() : 0.0);
            auto.addDouble((rightd.getBusVoltage() != 0.0)?rightd.getOutputVoltage()/rightd.getBusVoltage() : 0.0);
            auto.addDouble(kickerPower);
            auto.addBoolean(true);
            auto.addBoolean(roll.getOutputCurrent() > 15);
            auto.addDouble(0.0); // zone
            auto.addDouble(0.0); // Ball 1 X
            auto.addDouble(0.0); // Ball 1 Y
            auto.addDouble(0.0); // Ball 2 X
            auto.addDouble(0.0); // Ball 2 Y
            auto.addDouble(0.0); // Ball 3 X
            auto.addDouble(0.0); // Robot X
            auto.addDouble(0.0); // Robot Y
            auto.commit();
       }

       boolean blink = false;
       if((System.currentTimeMillis()-blinkStart) > 1500) // 1.5 sec
       {
           blink = true;
       }else if((System.currentTimeMillis()-blinkStart) > 3000)
       {
           blink = false;
           blinkStart = System.currentTimeMillis();
       }

       tele.addBoolean(gear);
       tele.addBoolean(true); // Target acquired
       tele.addBoolean(roll.getOutputCurrent() > 15);
       tele.addDouble(kickerPower);
       tele.addDouble(leftd.getOutputCurrent());
       tele.addDouble(rightd.getOutputCurrent());
       tele.addDouble(PSI);
       tele.addDouble(totalCurrent);
       tele.addBoolean((leftd.getBusVoltage() == 0.0)
                            ||((leftd.getFaults()!=0) && blink));
       tele.addBoolean((rightd.getBusVoltage() == 0.0)
                            ||((rightd.getFaults()!=0) && blink));
       tele.addBoolean((roll.getBusVoltage() == 0.0)
                            ||((roll.getFaults() != 0) && blink));
       tele.addBoolean((kicke.getBusVoltage() == 0.0)
                            ||((kicke.getFaults() != 0) && blink));
       tele.commit();
    }

}