package ca.mcgill.ecse211.localization;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.ColorDetector;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LightLocalization {
	private final int FOWARD_SPEED=80;
	//offset from center of wheel to lightsensor
	private final double OFFSET= 7;
	private Odometer odometer;
	private SampleProvider colorSensor;
	private float[] colorSample;
	private final int TANGLE=30;
	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;
	private Navigation navigation;
	private TextLCD LCD = LocalEV3.get().getTextLCD();

	/**
	 * Constructor of the class LightLocalization 
	 * uses the following parameters to locate the position of the robot, and move the robot 
	 * to the origin of the grid 
	 * @param odometer
	 * @param colorSample
	 * @param colorSensor
	 * @param leftMotor
	 * @param rightMotor
	 */
	public LightLocalization(Odometer odometer, float[] colorSample, SampleProvider colorSensor,
			EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor ) {
		this.odometer=odometer;
		this.colorSensor=colorSensor;
		this.colorSample=colorSample;
		this.leftMotor=leftMotor;
		this.rightMotor=rightMotor;
		this.leftMotor.setAcceleration(300);
		this.rightMotor.setAcceleration(300);
		navigation =new Navigation(leftMotor,rightMotor,odometer);

	}
	/**
	 * run method that move the robot to the origin
	 * it uses the lightSensor to localize its position on the grid, and travels to the origin
	 * @return nothing
	 */
	public void doLocalization() {

		int counter=0;

		do {
			navigation.turnTo(TANGLE);
			while(counter<2) {
				leftMotor.forward();
				rightMotor.forward();
				if(getColorChange()) {
					Sound.beep();
					counter=counter+1;
				}
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
			navigation.turnTo(-TANGLE);
			odometer.setPosition(new double [] {0.0, 0.0, 0}, new boolean [] {true, true, true});

			double x, y, theta_x, theta_y;
			double theta_1=0;
			double theta_2=0;
			double theta_3=0;
			double theta_4=0;

			boolean firstLine=true;
			boolean secondLine=true;
			boolean thirdLine=true;
			boolean fourthLine=true;


			leftMotor.setSpeed(FOWARD_SPEED);
			rightMotor.setSpeed(FOWARD_SPEED);
			//rotate clockwise
			leftMotor.forward();
			rightMotor.backward();
			while(firstLine) {
				if(getColorChange()) {
					Sound.beep();

					//keep note of the theta when light sensor first intersect the positive y-axis
					theta_1=odometer.getTheta();
					firstLine=false;
				}
			}
			while(secondLine) {
				if(getColorChange()) {
					Sound.beep();

					//keep note of the theta when light sensor intersect the positive x-axis
					theta_2=odometer.getTheta();
					secondLine=false;
				}
			}
			while(thirdLine) {
				if(getColorChange()) {
					Sound.beep();
					//keep note of the theta when light sensor first intersect the negative y-axis
					theta_3=odometer.getTheta();
					thirdLine=false;
				}
			}
			while(fourthLine) {
				if(getColorChange()) {
					Sound.beep();
					//keep note of the theta when light sensor intersect the negative x-axis
					theta_4=odometer.getTheta();
					fourthLine=false;
				}
			}
			//stop the robot when it catches the last line
			leftMotor.stop();
			rightMotor.stop();

			theta_x=Math.abs(theta_4-theta_2);
			theta_y=Math.abs(theta_3-theta_1);
			LCD.clear();

			x=-OFFSET*Math.cos(theta_y*Math.PI/360);
			y=-OFFSET*Math.cos(theta_x*Math.PI/360);

			LCD.drawString("X="+x, 0, 6);
			LCD.drawString("Y="+y, 0, 7);
			LCD.drawString("Theta1:"+theta_1,0, 5);
			LCD.drawString("Theta2"+theta_2, 0, 4);
			odometer.setX(x);
			odometer.setY(y);
			
			//the robot travels to the origin
			navigation.travelTo(0,0);
			//make the robot turn to 0 degree
			double nowT=odometer.getTheta();
			navigation.turnTo(-nowT);

		} while(Button.waitForAnyPress()!=Button.ID_ESCAPE);
		System.exit(0);
	}

	/**
	 * method that collects the data from the lightSensor and checks if there is a change in color 
	 * from a beige square to a black line
	 * @return boolean, if there is a change in the color, return true
	 * if there is not change, meaning still on the beige square, return false
	 */
	private boolean getColorChange() {
		colorSensor.fetchSample(colorSample, 0);
		boolean colorChange=false;
		int color= (int) colorSample[0] ;
		if(color==13) {
			colorChange=true;
		}
		return colorChange;
	}
}
