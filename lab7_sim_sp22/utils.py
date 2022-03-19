import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib import animation

class Simulator(object):
    """Object which simulates the EE16B car's controls model"""

    def __init__(self, theta_left, theta_right, beta_left, beta_right):
        self.theta = np.array([theta_left, theta_right])
        self.beta = np.array([beta_left, beta_right])

    def simulate(self,
                 v_star,
                 drive_straight_left,
                 drive_straight_right,
                 offset=0,
                 sim_length=10,
                 d0=(0, 0),
                 mismatch_error=0.1,
                 u_noise_factor=0.0):
        """Simulates the control model using the provided functions.

        Args:
            v_star (float): the desired velocity of the car.
            drive_straight_left (float, float -> float): function that computes
                the control output for the left wheel from v_star and delta.
            drive_straight_right (float, float -> float): function that computes
                the control output for the right wheel from v_star and delta.
            offset (float): the offset between the 2 wheels.
            sim_length (int): the length of the simulation.
            d0 (tuple of length 2): the initial distances of the left and right
                wheels.
            mismatch_error (float): how much error to introduce into the model parameters.

        Returns:
            d (sim_length x 2 np.array): the distance traveled of each wheel.
        """
        d = np.zeros([sim_length, 2])
        d[0] = d0
        # Deterministically mismatch model parameters so students are guaranteed to see effects
        theta = self.theta * (1 + mismatch_error * np.array([1, -1]))
        beta = self.beta * (1 + mismatch_error * np.array([1, -1]))

        for k in range(sim_length - 1):
            delta = d[k, 0] - d[k, 1] + offset
            if k < 1:
                # Jolt wheels with maximum power to start
                u = (255, 255)
            else:
                try:
                    u = np.array([
                        drive_straight_left(v_star, delta),
                        drive_straight_right(v_star, delta)
                    ])
                except TypeError:
                    u = np.array([
                        drive_straight_left(v_star),
                        drive_straight_right(v_star)
                    ])
            # Saturate inputs
            noise_amp = int(255*u_noise_factor/2)
            u = np.minimum(np.maximum(0, u +  np.random.randint(2*noise_amp+1,size=2) - noise_amp), 255)
            # Run a step of the system
            d[k + 1] = d[k] + theta * u - beta
            delta = d[:, 0] - d[:, 1]
            steady_state_error = delta[-1]

        return d, steady_state_error

    def plot(self, d, title):
        """Plots the results of the simulation.

        Args:
            d (np.array of shape sim_length x 2): the distance traveled of each
                wheel.
            title (string): title name

        Returns:
            delta (np.array of shape sim_length): the position difference
                between the left and right wheels.
        """
        delta = d[:, 0] - d[:, 1]
        sim_length = d.shape[0]
        # Post-process delta to determine stability
        ss_error = delta[-1]
        if abs(ss_error - delta[-2]) > 1:
            if abs(ss_error - delta[-3]) < 1e-1:
                ss_error = 'oscillatory'
            else:
                ss_error = "unbounded"
        elif abs(ss_error) < 1:
            ss_error = 0
        try:
            print("Steady state error = {:.2f}".format(ss_error))
        except ValueError:
            print("Steady state error = {}".format(ss_error))
            
        plt.figure(figsize=(12, 5))
        # plot distance
        plt.subplot(121)
        plt.plot(range(sim_length), d[:, 0], 'b-', range(sim_length), d[:, 1], 'y-')
        plt.xlabel("i (sample)")
        plt.ylabel("Distance")
        plt.legend(("left", "right"), loc="best")
        plt.grid()
        
        # plot delta  
        plt.subplot(122)    
        plt.plot(delta, 'r')
        plt.xlabel("i (sample)")
        plt.ylabel('delta')
        plt.grid()
        
        # title
        plt.suptitle(title)
        return delta

def plot_car_traj_single(  title,
                    simulator,
                    v_star,
                    drive_straight_left,
                    drive_straight_right,
                    offset=0,
                    sim_length=10,
                    d0=(0, 0),
                    mismatch_error=0.1,
                    u_noise_factor=0.0):
    L = 60      # distance between left and right wheel, aka the width of the car
    d, _ = simulator.simulate(v_star,
                              drive_straight_left,
                              drive_straight_right,
                              offset=offset,
                              sim_length=sim_length,
                              d0=d0,
                              mismatch_error=mismatch_error,
                              u_noise_factor=u_noise_factor)
    delta_d_left = np.diff(d[:,0])
    delta_d_right = np.diff(d[:,1])
    theta = (delta_d_left - delta_d_right) / L / np.pi * 180
    theta = np.cumsum(theta)
    sim_length = theta.shape[0]
    
    x = np.zeros((sim_length,))
    y = np.zeros((sim_length,))
    for i in range(sim_length):
        if i != 0:
            y[i] = y [i-1] - np.sin(theta[i-1] * np.pi / 180) * v_star
            x[i] = x [i-1] + np.cos(theta[i-1] * np.pi / 180) * v_star
    
    return x,y


def plot_car_traj(title_arr,
                  simulator,
                  v_star,
                  drive_straight_left,
                  drive_straight_right,
                  offset=0,
                  sim_length=10,
                  d0=(0, 0),
                  mismatch_error_arr=[0.0],
                  u_noise_factor_arr=[0.0]):
    colors = iter([plt.cm.tab10(i) for i in range(10)])
    fig = plt.figure(figsize=(12, 12))
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')
    plt.xlabel("x")
    plt.ylabel("y")
    for title, mismatch_error, u_noise_factor in zip(title_arr, mismatch_error_arr, u_noise_factor_arr):
        _offset = 0 if (mismatch_error == 0) else offset
        x,y = plot_car_traj_single(title,
                simulator,
                v_star,
                drive_straight_left,
                drive_straight_right,
                offset=_offset,
                sim_length=sim_length,
                d0=(0, 0),
                mismatch_error=mismatch_error,
                u_noise_factor=u_noise_factor)
        ax.quiver(x[:-1], y[:-1], x[1:]-x[:-1], y[1:]-y[:-1], scale_units='xy', angles='xy', scale=1, color=[next(colors)])

      
    plt.grid()
    plt.legend(title_arr, loc="best")
    plt.title('Trajectory of the car')
    return 

    

def two_sims(titles,
             simulator,
             v_star,
             drive_straight_left,
             drive_straight_right,
             offset=0,
             sim_length=10,
             d0=(0, 0),
             mismatch_error=0.1):
    
    d1, _ = simulator.simulate(v_star,
                               drive_straight_left,
                               drive_straight_right,
                               offset=offset,
                               sim_length=sim_length,
                               d0=d0,
                               mismatch_error=0)
    print('Perfect model :')
    simulator.plot(d1, titles[0])

    d2, _ = simulator.simulate(v_star,
                               drive_straight_left,
                               drive_straight_right,
                               offset=offset,
                               sim_length=sim_length,
                               d0=d0,
                               mismatch_error=mismatch_error)
    print('Non-ideal model :')
    simulator.plot(d2, titles[1])


def find_closest_index(arr, val):
    """Given an array of velocities and a value, returns the index of the array where
    arr[index] is the closest to the given value.
    """
    return np.array(abs(arr - val)).argmin()


def find_jolt(filename="data.txt"):
    data = np.genfromtxt(filename, dtype=np.int, delimiter=",")

    # Compute velocity from distances
    vleft = data[:, 1]
    vright = data[:, 2]
    u = data[:, 0].reshape((-1, 1))  # duty cycle

    left_des, _ = np.array_split(vleft, 2)
    right_des, _ = np.array_split(vright, 2)
    u_des, _ = np.array_split(u, 2)

    plt.figure()
    plt.axis((0, 250, 0, max(max(left_des), max(right_des)) + 10))
    max_vel = min(max(left_des), max(right_des))
    plt.plot(u_des, left_des, 'b', u_des, right_des, 'y')
    plt.axhline(max_vel, color='g', linestyle='--')
    plt.xlabel("u (input via PWM)")
    plt.ylabel("Velocity of Wheels")
    labels = ("left wheel", "right wheel", "max velocity")
    plt.legend(labels, loc=2)

    left_idx = find_closest_index(left_des, max_vel)
    right_idx = find_closest_index(right_des, max_vel)
    left_jolt = u_des[left_idx][0]
    right_jolt = u_des[right_idx][0]
    plt.axvline(left_jolt, color='b', linestyle='--')
    plt.axvline(right_jolt, color='y', linestyle='--')
    return left_jolt, right_jolt
