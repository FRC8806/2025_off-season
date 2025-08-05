package frc.robot;

public class Tools {
  public static double map(double measure, double mMin, double mMax, double oMin, double oMax) {
    return oMin + (oMax - oMin)*(measure - mMin)/(mMax - mMin);
  }

  public static double highSpeed(double value, double max) {
    return Math.max(Math.min(value, max), -max);
}
}
