package com.drone.sim;

import com.badlogic.gdx.math.Vector3;

public class CubicHermite3D {
    public Vector3 pos0, pos1, vel0, vel1;
    public float length;

    public CubicHermite3D(Vector3 pos0, Vector3 pos1, Vector3 vel0, Vector3 vel1) {
        this.pos0 = pos0;
        this.pos1 = pos1;
        this.vel0 = vel0;
        this.vel1 = vel1;

        this.length = getGaussianQuadratureLength(0, 1);

//        System.out.println(getGaussianQuadratureLength(0, 1));
//        float l = 0;
//        for(float k = 0.001f; k <= 1 + 1e-6; k += 0.001f) {
//            Vector3 p = get(k).sub(get(k-0.001f));
//            l += Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
//        }
//        System.out.println(l);
    }

    public Vector3 get(float t) {
        return this.get(t, 0);
    }

    public Vector3 get(float t, int nD) {
        if (t < 0) return pos0;
        if (t > 1) return pos1;
        return pos0.cpy().scl(basis(t, 0, nD))
                .add(pos1.cpy().scl(basis(t, 3, nD)))
                .add(vel0.cpy().scl(basis(t, 1, nD)))
                .add(vel1.cpy().scl(basis(t, 2, nD)));
    }

    public float basis(float t, int i, int nD) {
        switch (nD) {
            case 0: {
                switch (i) {
                    case 0: return 1 - 3 * t * t + 2 * t * t * t;
                    case 1: return t - 2 * t * t + t * t * t;
                    case 2: return -t * t + t * t * t;
                    case 3: return 3 * t * t - 2 * t * t * t;
                    default: return 0;
                }
            }
            case 1: {
                switch (i) {
                    case 0: return -6 * t + 6 * t * t;
                    case 1: return 1 - 4 * t + 3 * t * t;
                    case 2: return -2 * t + 3 * t * t;
                    case 3: return 6 * t - 6 * t * t;
                    default: return 0;
                }
            }
            case 2: {
                switch (i) {
                    case 0: return -6 + 12 * t;
                    case 1: return -4 + 6 * t;
                    case 2: return -2 + 6 * t;
                    case 3: return 6 - 12 * t;
                    default: return 0;
                }
            }
            default: return 0;
        }
    }

    public double[][] getGaussianCoefs() {
        return new double[][] {
                {0.179446470356207, 0.0000000000000000},
                {0.176562705366993, -0.178484181495848},
                {0.176562705366993, 0.178484181495848},
                {0.1680041021564500, -0.351231763453876},
                {0.1680041021564500, 0.351231763453876},
                {0.15404576107681, -0.512690537086477},
                {0.15404576107681, 0.512690537086477},
                {0.135136368468526, -0.657671159216691},
                {0.135136368468526, 0.657671159216691},
                {0.1118838471934040, -0.781514003896801},
                {0.1118838471934040, 0.781514003896801},
                {0.0850361483171792, -0.880239153726986},
                {0.0850361483171792, 0.880239153726986},
                {0.0554595293739872, -0.950675521768768},
                {0.0554595293739872, 0.950675521768768},
                {0.0241483028685479, -0.990575475314417},
                {0.0241483028685479, 0.990575475314417}
        };
    }

    public float getGaussianQuadratureLength(float start, float end) {
        double[][] coefficients = getGaussianCoefs();

        //we are trying to find integral of sqrt(x'(t)^2 + y'(t)^2) from start to end

        //integral bound transformation from [0,1] to [-1, 1]
        double half = (end - start) / 2.0;
        double avg = (start + end) / 2.0;
        double length = 0;
        for (double[] coefficient : coefficients) {
            //sqrt(x'(t)^2 + y'(t)^2)
            length += get((float) (avg + half * coefficient[1]), 1).len() * coefficient[0];
        }
        return (float) (length * half);
    }

    public Vector3 getClosestPoint(Vector3 point) {
        return get(findClosestPointOnSpline(point, 50, 5));
    }

    public float findClosestPointOnSpline(Vector3 point) {
        return this.findClosestPointOnSpline(point, 50, 5);
    }

    public float findClosestPointOnSpline(Vector3 point, int steps, int iterations) {

        double cur_dist = Double.POSITIVE_INFINITY;
        double cur_min = 0;

        //the steps to start Newton's method from
        for(double i = 0; i <= 1; i += 1./steps) {
            float cur_t = (float) i;

            //amount to adjust according to Newton's method
            //https://en.wikipedia.org/wiki/Newton%27s_method
            //using first and second derivatives because we want min of distance function (zero of its derivative)
            double dt = getFirstDerivAtT(cur_t, point) /
                        getSecondDerivAtT(cur_t, point);

            int counter = 0;

            //run for certain number of iterations
            while(counter < iterations) {

                //adjust based on Newton's method, get new derivatives
                cur_t -= dt;
                dt = getFirstDerivAtT(cur_t, point) /
                     getSecondDerivAtT(cur_t, point);
                counter++;
            }

            //if distance is less than previous min, update distance and t
            double cur_d = get(cur_t).dst(point);

            if(cur_d < cur_dist) {
                cur_dist = cur_d;
                cur_min = cur_t;
            }
        }

        //return t of minimum distance, clamped from 0 to 1
        return (float) Math.min(1, Math.max(0, cur_min));

    }

    public double getFirstDerivAtT(float t, Vector3 point) {
        Vector3 p = get(t);
        Vector3 d1 = get(t, 1);

        double x_a = p.x - point.x;
        double y_a = p.y - point.y;
        double z_a = p.z - point.z;

        return 2 * (x_a * d1.x + y_a * d1.y + z_a * d1.z);
    }

    public double getSecondDerivAtT(float t, Vector3 point) {
        Vector3 p = get(t);
        Vector3 d1 = get(t, 1);
        Vector3 d2 = get(t, 2);

        double x_a = p.x - point.x;
        double y_a = p.y - point.y;
        double z_a = p.z - point.z;

        return 2 * (d1.x * d1.x + x_a * d2.x + d1.y * d1.y + y_a * d2.y + d1.z * d1.z + z_a * d2.z);
    }

    public float getTFromLength(float length) {
        float t = length / this.length;

        for(int i = 0; i < 5; i++) {
            //magnitude of the derivative
            float derivativeMagnitude = get(t, 1).len();

            //Newton's method: length remaining length divided by derivative
            if(derivativeMagnitude > 0.0) {
                t -= (getGaussianQuadratureLength(0, t) - length) / derivativeMagnitude;
                //Clamp to [0, 1]
                t = Math.min(1, Math.max(t, 0));
            }
        }

        return t;
    }
}
