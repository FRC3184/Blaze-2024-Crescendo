package frc.robot.Subsystems;

import edu.wpi.first.math.Pair;
import frc.Mechanisms.pivot.*;
import frc.robot.SubmoduleSubsystemConstants.ConstShooter;

public class ShooterPitch extends pivotOneMotorRevNeoAbsEncoder {
  // corresponds to the values theta1, and r1 through r4 on this diagram
  // http://www.softintegration.com/chhtml/toolkit/mechanism/fig/fourbar/fourbarvel_trans.gif
  static final double t1 = 0;
  static final double r1 = 1;
  static final double r2 = 1;
  static final double r3 = 1;
  static final double r4 = 1;

  public ShooterPitch() {
    super(ConstShooter.kPivot, ConstShooter.invertedPivot, ConstShooter.velFactorPivot, "Shot Pitch", 1, 0, 0);
  }

  // used this source to help derive this
  // https://www.softintegration.com/chhtml/toolkit/mechanism/fourbar/fourbarpos.html
  // output is in revolutions
  public Pair<Double, Double> inputAngleFromRockerAngle(double rockerAngle) {

    double x = Math.cos(t1) + Math.cos(rockerAngle);
    double y = Math.sin(t1) + Math.sin(rockerAngle);

    double acosPart = Math.acos(((x*x)+(y*y)+(r2*r2)-(r3*r3))/(2*r2*Math.sqrt((x*x)+(y*y))));
    double theta1 = Math.atan2(y, x) + acosPart;
    double theta2 = Math.atan2(y, x) - acosPart;

    return Pair.of(theta1, theta2);
  }
}