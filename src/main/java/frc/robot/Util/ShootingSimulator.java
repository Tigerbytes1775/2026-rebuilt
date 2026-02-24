package frc.robot.Util;




public class ShootingSimulator {

    private final double marginOfError = 0.0001;
    
    public ShootingSimulator() {}

    public double[] simShot(double launchSpeed, double angle, double incline, double time,  double[] robotVel, double[] robotPos) {
        
        double[] result = {
            (Math.cos(incline) * Math.cos(angle) * launchSpeed + robotVel[0]) * time + robotPos[0],
            (Math.cos(incline) * Math.sin(angle) * launchSpeed + robotVel[1]) * time + robotPos[1],
            (Math.sin(incline) * launchSpeed - 4.905 * time) * time + robotPos[2]
        };

        return result;
    }

    public double getDistance(double[] a, double[] b) {
        double[] distanceVector = new double[3];

        for(int i = 0; i < 3; i++) {
            distanceVector[i] = a[i] - b[i];
        }

        return Math.sqrt(Math.pow(distanceVector[0],2) + Math.pow(distanceVector[1], 2) + Math.pow(distanceVector[2], 2));
        
    }

    public boolean checkShot(double launchSpeed, double angle, double incline, double time, double[] robotVel, double[] robotPos, double[] target) {
        double[] result = simShot(launchSpeed, angle, incline, time, robotVel, robotPos);
        return getDistance(target, result) < marginOfError;
    }

    public boolean checkShot(double launchSpeed, double angle, double incline, double[] robotVel, double[] robotPos, double[] target, double marginOfError) {

        boolean inAir = true;
        double time = 0.1;
        while(inAir) {

            double[] result = simShot(launchSpeed, angle, incline, time, robotVel, robotPos);
            time += 0.005;
            if (getDistance(result, target) < marginOfError) {
                return true;
            }

            inAir = result[2] >= 0;
        }

        return false;

    }

}
