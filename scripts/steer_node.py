#!/usr/bin/python

class Driver:
    def steer(self, x_hat_1):
        if self.waypoints:
            cwp = self.waypoints[0]
            x = x_hat_1[0, 0]
            y = x_hat_1[1, 0]
            theta = x_hat_1[2, 0]
            cpos = Point(x, y)
            dist = distance(cpos, cwp)
            dx = cwp.x - cpos.x
            dy = cwp.y - cpos.y
            t_angle = np.arctan2(dy, dx)
            dangle = clipangle(t_angle - theta)
            u = 0.1*np.matrix([3.0, 4.0 * dangle]).transpose()
            if dist < 0.7:
                print('At waypoint!', self.waypoints.pop(0))
        else:
            u = np.matrix([[0.0], [0.0]])
        return u


if __name__ == '__main__':
    driver = Driver()
    driver.steer()
