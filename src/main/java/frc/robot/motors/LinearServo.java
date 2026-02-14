package frc.robot.motors;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;

public class LinearServo extends Servo
{
    double m_speed;
    double m_length;
    double setPos;
    double curPos;
    double maxExtension;
    double minExtension;
    /**
    * Parameters for L16-R Actuonix Linear Actuators
    *
    * @param channel PWM channel used to control the servo
    * @param length max length of the servo [mm]
    * @param speed max speed of the servo [mm/second]
    */
    public LinearServo(int channel, int length, int speed, double maxExtension, double minExtension) 
    {
        super(channel);
        setBoundsMicroseconds(2000, 1550, 1500, 1450, 1000);
        m_length = length;
        m_speed = speed;
        this.maxExtension = maxExtension;
        this.minExtension = minExtension;
    }

    /**
     * Run this method in any periodic function to update the position estimation of your servo
    * @param setpoint the target position of the servo [mm]
    */
    public void setPosition(double setpoint)
    {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed( (setPos/m_length *2)-1);
    }
    double lastTime = 0;
   
    /**
     * Run this method in any periodic function to update the position estimation of your servo
    */
    public void updateCurPos()
    {
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed * dt)
        {
            curPos -= m_speed * dt;
        } 
        else if(curPos < setPos - m_speed * dt)
        {
            curPos += m_speed * dt;
        }
        else
        {
            curPos = setPos;
        }
    }

    /**
     * Current position of the servo, must be calling {@link #updateCurPos() updateCurPos()} periodically
    * @return Servo Position [mm]
    */
    public double getPosition()
    {
        return curPos;
    }

    // these methods need to use the PWM setPosition so comment out the
    // LinearServo setPosition() if you intend to use
    // public void fullyExtend()
    // {
    //     setPosition(0.8);
    // }

    // public void fullyRetract()
    // {
    //     setPosition(1.0);
    // }

    // extends the servo to maxExtension
    public void extend()
    {
        setPosition(maxExtension);
    }

    // 
    public void retract()
    {   
        setPosition(minExtension);
    }

    /**
     * Checks if the servo is at its target position, must be calling {@link #updateCurPos() updateCurPos()} periodically
    * @return true when servo is at its target
    */
    public boolean isFinished()
    {
        return curPos == setPos;
    }
}