package ca.mcgill.ecse211.navigationlab;

import lejos.hardware.Button;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Navigation extends Thread {

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private EV3MediumRegulatedMotor usMotor;
	private TextLCD t;
	private Odometer odometer;
	private SampleProvider us;
	private float[] usData;

	//start a 2D array of waypoints
	private static int[][] waypoints=new int[][]{{1,0},{2,2},{2,1}};

	private static final int FORWARD_SPEED = 180;
	private static final int ROTATE_SPEED = 180;
	private static final double WHEEL_RADIUS=2.2;
	private static final double TRACK= 9.88;	//15.755, 9.88
	private static final double SQRTLENGTH = 30.48;		//for converting on the grid
	private static final int OFFSET = 2;
	private static final int motorLow = 80;
	private static final int motorHigh = 260;
	private static final int SPECIFICDIST = 5;

	private double turntheta;
	private boolean isAvoiding;
	private boolean navigating;

	private double x;
	private double y;
	private double thetaAvoid=0;
	private double finishTheta=0;
	private int bandCenter=35;
	private int bandWidth=9;

/**
 * This the constructor of the class Navigation that takes in the 7 parameters below
 * @param leftMotor
 * @param rightMotor
 * @param odometer
 * @param t
 * @param us
 * @param usData
 * @param usMotor
 */
	
	public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, 
			Odometer odometer, TextLCD t, SampleProvider us, float[] usData, EV3MediumRegulatedMotor usMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.odometer=odometer;
		this.t=t;
		this.us=us; 
		this.usData=usData;
		this.usMotor=usMotor;
	}

	/**
	 * this method is the run method that will navigate to the coordinates of the waypoints array
	 * this checks the boolean isObstacle and enters a different loop depending on the selection in the main
	 * if isObstacle ==true, it will go directly to the points in the array waypoints
	 * if isObstacle == false, it will continuously update the data and the ultrasonic sensor will
	 * detect if there is an obstacle not not, and move accordingly
	 * 
	 * @return nothing
	 */
	public void run() {
		int currentIndex = 0;
		int maxIndex = waypoints.length;
		int error;
		int distance;

		if(NavigationObstacleAvoidanceLab.isObstacle==true) {
			isAvoiding = false;
			while(true) {
				us.fetchSample(usData, 0);
				distance = (int) (usData[0] *100); //takes value from the buffer in cm
				error=this.bandCenter-distance;


				if(isAvoiding==true) {
					avoid(error, distance);
				}

				//first time it sees a wall
				else if((error>bandWidth) && isAvoiding==false) {
					thetaAvoid=odometer.getTheta();
					finishTheta=thetaAvoid+90;

					//normalization , make sure theta is between 0 and 360 degrees
					if (finishTheta >= 360) {
						finishTheta = finishTheta - 360;
					}
					else if( finishTheta < 0) {
						finishTheta = finishTheta +360;
					}
					avoid(error, distance);
					isAvoiding=true;
				}

				//no obstacle found
				else
				{
					leftMotor.setSpeed(FORWARD_SPEED);
					rightMotor.setSpeed(FORWARD_SPEED);


					//move to the position
					x=waypoints[currentIndex][0]*SQRTLENGTH;
					y=waypoints[currentIndex][1]*SQRTLENGTH;
					turntheta=getAngle(x,y);
					turnTo(turntheta);
					goForward(SPECIFICDIST);
					
					//check if it has reached all the points
					if(Math.abs(x-odometer.getX())<OFFSET && Math.abs(y-odometer.getY())<OFFSET) {
						if(currentIndex<maxIndex) {
							currentIndex = currentIndex + 1;
						}
					}
				}
			}

		}
		//if the LEFT button is pressed, will go through this loop that does not take in consideration the possible obstacles
		else if (NavigationObstacleAvoidanceLab.isObstacle == false) {
			for(int i=0; i<waypoints.length; i++) {
				double x= waypoints[i][0]*SQRTLENGTH;
				double y=waypoints[i][1]*SQRTLENGTH;
				travelTo(x,y);
			}
		}
	}

	/**
	 * This method is used to avoid any obstacle it finds in its way
	 * depending on the error and the distance at which the robot is
	 * the robot will use BangBang to avoid the block by moving clockwise around the block
	 * if the robot first enter the avoid method, it will set the ultrasonic sensor to a 45 degree angle
	 * so it can see the wall, and place it back in the middle once it has done avoidance
	 * @param error
	 * @param distance
	 * @return nothing
	 */
	public void avoid(int error, int distance) {

		if(error>bandWidth) {
			leftMotor.setSpeed(motorLow);
			rightMotor.setSpeed(motorHigh);
			leftMotor.forward();
			rightMotor.forward();
		}
		else if(error<-bandWidth) {
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorLow);
			leftMotor.forward();
			rightMotor.forward();
		}

		if(isAvoiding==false) {
			usMotor.rotate(-45);
		}

		if((odometer.getTheta()-OFFSET)<finishTheta && finishTheta<(odometer.getTheta()+OFFSET)) {
			turnTo(finishTheta);
			isAvoiding=false;
			usMotor.rotate(45);
		
		}
	}

	/**
	 * This method takes in x and y and calculates the angle at which it should move
	 * and return this angle in degree
	 * @param x
	 * @param y
	 * @return turnTheta in degree [0, 360)
	 */
	public double getAngle(double x, double y) {
		double x_diff=x-odometer.getX();
		double y_diff=y-odometer.getY();
		double dtheta=Math.atan2(x_diff,y_diff)*180/Math.PI;
		double nowtheta=odometer.getTheta();
		double turntheta=dtheta-nowtheta;

		if(turntheta<-180) {
			turntheta=turntheta+360;
		}

		else if(turntheta>180) {
			turntheta=turntheta-360;
		}

		return turntheta;
	}

	/**
	 * This method causes the robot to travel to the absolute field location(x,y),
	 * specified in tile points. This will make sure that the heading
	 * is updated until you reach your exact goal. 
	 * This method will poll the odometer for current position information.
	 * @param x
	 * @param y
	 * @return nothing
	 */
	public void travelTo(double x, double y) {
		double x_diff=x-odometer.getX();
		double y_diff=y-odometer.getY();
		double dtheta=Math.atan2(x_diff,y_diff)*180/Math.PI;
		double nowtheta=odometer.getTheta();
		double turntheta=dtheta-nowtheta;
		double distance=Math.sqrt(Math.pow(x_diff, 2)+Math.pow(y_diff, 2));
		//display difference between x and nowx, y and nowy on screen

		t.drawString("x_diff:"+x_diff, 0, 6);
		t.drawString("y_diff:"+y_diff, 0, 7);
		t.drawString("distance:"+distance, 0, 8);
		t.drawString("turntheta:"+turntheta, 0, 5);

		if(turntheta<-180) {
			turntheta=turntheta+360;
		}

		else if(turntheta>180) {
			turntheta=turntheta-360;
		}

		turnTo(turntheta);
		goForward(distance);

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
		navigating=true;
		leftMotor.rotate(angleConvert(WHEEL_RADIUS,TRACK,theta),true);
		rightMotor.rotate(-angleConvert(WHEEL_RADIUS,TRACK,theta),false);
		navigating=false;
	}

/**
 * This method goes forward for a specific distance d
 * This method calls rotate and distanceConvert(radius, distance) to have the value in tachocount
 * @param d
 * @return nothing
 */
	public void goForward(double d) {
		leftMotor.setSpeed(FORWARD_SPEED);
		rightMotor.setSpeed(FORWARD_SPEED);
		leftMotor.forward();
		rightMotor.forward();

		navigating=true;
		leftMotor.rotate(distanceConvert(WHEEL_RADIUS,d),true);
		rightMotor.rotate(distanceConvert(WHEEL_RADIUS,d),false);
		navigating=false;
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
