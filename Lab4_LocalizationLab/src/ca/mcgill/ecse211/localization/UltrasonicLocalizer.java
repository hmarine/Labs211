package ca.mcgill.ecse211.localization;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


public class UltrasonicLocalizer {
	
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private TextLCD LCD=LocalEV3.get().getTextLCD();
	private Odometer odometer;
	private SampleProvider us;
	private float[] usData;

	private static final int ROTATE_SPEED=70;
	private static final double WHEEL_RADIUS = 2.2;
	private static final double TRACK = 9.88;
	private static final int D=50;
	private static int distance;
	private static int dT;
	private double thetaA, thetaB;


	/**
	 * Constructor for the class UltrasonicLocalizer, that uses the robot's motors to rotate on itself, and the odometer
	 * to know its position related to the grid. It localizes its surroundings by collecting data in an array.
	 * @param leftMotor, Motor for the left wheel
	 * @param rightMotor, Motor for the right wheel
	 * @param odometer, uses odometry to display the position of the robot
	 * @param us, sample provider
	 * @param usData, array that collects the data
	 */
	public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			Odometer odometer, SampleProvider us, float[] usData) {
		this.leftMotor=leftMotor;
		this.rightMotor=rightMotor;
		this.odometer=odometer;
		
		this.us=us;
		this.usData=usData;

	}

	/**
	 * Main method that runs the program once the LocalizationLab starts the program.
	 * This is activated by a button press in the LocalizationLab, and will run doLocalization.
	 * doLocalization calls two different methods depending on the press in LocalizationLab,
	 * fallingEdge() and risingEdge()
	 * it will execute one of the following
	 */
public void doLocalization() {
		//this is a falling edge 
		if(LocalizationLab.getFallingEdge()==true) {
			fallingEdge();
		}
		//this is a rising edge
		else if(LocalizationLab.getFallingEdge()==false) {
			risingEdge();
		}
		}
	

	//collecting data for the distances
	public int collectData() {
		us.fetchSample(usData, 0);
		distance = (int) (usData[0] *100);//takes value from the buffer in cm
		
		if(distance>D) {
			distance=D;
		}
		return distance;
	}

	/**
	 * Method that detects the rising edge, from where the value for the distance is small, and then becomes very large
	 * since there are no wall detected anymore.
	 * this is the rising point. It uses the rising edge to localize itself in space and time and redirect itself,
	 * to the 0 degree angle, facing the positive y-axis
	 * @return nothing
	 */
	public void risingEdge() {
		
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		//rotate clockwise, until it sees the wall
		while(collectData()==D) {
			LCD.drawString("Distance:"+collectData(), 0, 4);
			LCD.drawString("Step 1", 0, 5);
			leftMotor.forward();
			rightMotor.backward();
		}
		//continue until it doesn't see the wall
		leftMotor.forward();
		rightMotor.backward();
		
		boolean isTurning=true;
		while(isTurning) {
			LCD.drawString("Distance:"+collectData(), 0, 4);
			LCD.drawString("Step 2", 0, 5);
			if(collectData()==D) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				isTurning=false;
			}
		}
		Sound.beep();


		thetaA=odometer.getTheta();
		thetaA=normalizeTheta(thetaA);
		Sound.beep();
		
		//change direction until it doesn't see the wall
	
		while(collectData()==D) {
			LCD.drawString("Distance"+collectData(), 0, 4);
			LCD.drawString("Step 3", 0, 5);
			leftMotor.backward();
			rightMotor.forward();
		}
		//keep rotation until robot sees the wall, note down the angel
		leftMotor.backward();
		rightMotor.forward();
		
		isTurning=true;
		
		while(isTurning){
			LCD.drawString("Distance"+collectData(), 0, 4);
			LCD.drawString("Step 4", 0, 5);
			if(collectData()==D) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				isTurning=false;
			}
		}
		Sound.beep();
		thetaB=odometer.getTheta();
		thetaB=normalizeTheta(thetaB);
		
		LCD.drawString("ThetaA:"+thetaA, 0, 6);
		LCD.drawString("ThetaB"+thetaB, 0, 7);
		
		if(thetaA<thetaB) {
			dT= 45-(int) (thetaA+thetaB)/2;
		}
		else if(thetaA>thetaB) {
			dT=225-(int) ((thetaA + thetaB) /2);
		}
		LCD.drawString("DT:"+dT, 0, 3);
		double currentTheta= odometer.getTheta();
		double newtheta= dT+currentTheta;
		LCD.drawString("Step 5", 0, 5);
		turnTo(360-newtheta);	
	}

	/**
	 * Method that uses the falling edge, where the distance seen by the robot is very large, as it is facing no wall,
	 * and falling as it turns and finds a wall. This is the falling point
	 * It uses the falling points to localize itself in space and time, to be at the 0 degree angle, which is facing
	 * the positive y-axis
	 * 
	 * @return nothing
	 */
	public void fallingEdge() {
		
		//set rotation speed
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
		
		//rotate clockwise until it sees no wall
		while(collectData()<D) {
			LCD.drawString("Distance"+collectData(), 0, 4);
			LCD.drawString("Step 1", 0, 5);
			leftMotor.forward();
			rightMotor.backward();
		}

			//keep rotating until the robot sees the wall,then get the angel
			leftMotor.forward();
			rightMotor.backward();
			boolean isTurning=true;
	
			while(isTurning) {
				LCD.drawString("Distance"+collectData(), 0, 4);
				LCD.drawString("Step 2", 0, 5);
				if(collectData()<D) {
					leftMotor.stop(true);
					rightMotor.stop(false);
					isTurning=false;
				}
			}
		Sound.beep();
		thetaA=odometer.getTheta();
		thetaA=normalizeTheta(thetaA);
		
		//switch direction until it sees no wall
		while(collectData()<D) {
			LCD.drawString("Distance"+collectData(), 0, 4);
			LCD.drawString("Step 3", 0, 5);
			leftMotor.backward();
			rightMotor.forward();
		}
		
		//keep rotation until the robot sees the wall
		leftMotor.backward();
		rightMotor.forward();
		
		isTurning=true;
		while(isTurning) {
			LCD.drawString("Distance"+collectData(), 0, 4);
			LCD.drawString("Step 4", 0, 5);
			if(collectData()<D) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				isTurning=false;
			}
		}
		Sound.beep();
		
		thetaB=odometer.getTheta();
		thetaB=normalizeTheta(thetaB);
		Sound.beep();
		LCD.drawString("ThetaA:"+thetaA, 0, 6);
		LCD.drawString("ThetaB"+thetaB, 0, 7);
		
		if(thetaA>thetaB) {
			dT= 45-(int) (thetaA+thetaB)/2;
		}
		else if(thetaA<thetaB) {
			dT= 225-(int) (thetaA+thetaB)/2;
		}
		LCD.drawString("DT:"+dT, 0, 3);
		double currentTheta= odometer.getTheta();
		double newtheta=currentTheta+dT;
		LCD.drawString("Step 5", 0, 5);
		turnTo(360-newtheta);
	}
	
	/**
	 * Method that constraint the values of the thetas to be in degree between the values of [0, 360)
	 * 
	 * @param theta which is the angle in degree
	 * @return theta, which is the angle in degree between [0,360)
	 */
	public double normalizeTheta(double theta) {
		if (theta >= 360) {
			theta = theta - 360;
		}
		else if( theta < 0) {
			theta = theta +360;
		}
		return theta;
	}

	/**
	 *  Method that causes the robot to turn on itself to the absolute heading theta. 
	 * This method should turn a MINIMAL angle to its target.
	 * @param theta
	 * @return Nothing
	 */
	void turnTo(double theta) {
		leftMotor.setSpeed(ROTATE_SPEED);
		rightMotor.setSpeed(ROTATE_SPEED);
	
		leftMotor.rotate(angleConvert(WHEEL_RADIUS,TRACK,theta),true);
		rightMotor.rotate(-angleConvert(WHEEL_RADIUS,TRACK,theta),false);
		
	}

	/**
	 * Method returns the number of tachocount it need to travel a certain distance
	 * Calculates the tachocount from the radius of the wheel
	 * @param radius
	 * @param distance
	 * @return
	 */
	public static int distanceConvert(double radius, double distance) {
		return(int) (distance*180/Math.PI/radius);
	}


	/**
	 * This method converts the angle into the number of tachocount it need to turn for this degree
	 * @param radius
	 * @param width
	 * @param angle
	 * @return an integer that is the distance
	 */
	private static int angleConvert(double radius, double width, double angle) {

		return distanceConvert(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method is to know whether or not the robot is traveling
	 * 
	 * @return Boolean isTravelling
	 */
	boolean isNavigating() {
		boolean isTravelling = true;
		return isTravelling;
	}
}
