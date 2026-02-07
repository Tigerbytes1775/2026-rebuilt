package frc.robot.Util;

public class Aimer {
    
    ShootingSimulator shooterSim;
    public Aimer() {
        shooterSim = new ShootingSimulator();
    }

    //gets the x-y distance to target
    private double getDistance(double[] target) {
        return Math.sqrt(target[0] * target[0]+ target[1]* target[1]);
    }

    private double getTime(double launchSpeed, double distance, double incline) {
        return distance / (launchSpeed * Math.cos(incline));
    }

    private double getTurretAngle(double[] target) {
        return Math.atan2(target[1],target[0]);
    }

    private double getLaunchSpeed(double distance, double height, double incline) {
        return Math.sqrt(-(9.81*distance*distance)/((2*Math.cos(incline)*Math.cos(incline)) * (height-distance*Math.tan(incline))));
    }


    /*
     * returns information on how to aim a shot based of the parametres.
     * returns an array formated as shown: {time,launchSpeed, turretAngle}
     */
    public double[] aimShot(double incline, double[] pos, double[] target, double[] robotVel) {
        boolean inaccurate = true;
        double time = 0;
        double launchSpeed= 0;
        double angle = 0;
        int iterations = 0;
        while (inaccurate) {
            iterations += 1;

            System.out.println(iterations);
            double[] difference = new double[3];

            for(int i = 0; i < 3; i++) {
                difference[i] = target[i] - pos[i] - robotVel[i] * time;
            }

            angle = getTurretAngle(difference);
            double distance = getDistance(difference);
            launchSpeed = getLaunchSpeed(distance, difference[2], incline);

            time = getTime(launchSpeed, distance, incline);


            inaccurate = !shooterSim.checkShot(launchSpeed, angle, incline, time, robotVel, pos, target);

        }
        double[] result = {time,launchSpeed,angle};
        return result;

    }


}
