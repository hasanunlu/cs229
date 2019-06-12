import gym
import numpy as np
import matplotlib.pyplot as plt

env = gym.make('CartPole-v1')
#env = TraceRecordingWrapper(env)

def accelermeter_mode(theta, theta_dot, theta_dot_old, x_dot, x_dot_old):
    # Assume that accelerometer is placed in middle of the pole.
    # measured axis are -y and +x, -x
    # state updates are 0.02 seconds
    delta_t = 0.02
    acc_position = 0.5
    x_acc = (x_dot - x_dot_old) / delta_t
    linear_acc_in_y = acc_position * (theta_dot*theta_dot)
    gravity_x = 9.8 * np.sin(theta)
    gravity_y = 9.8 * np.cos(theta)
    x_acc_in_x = x_acc * np.cos(theta)
    x_acc_in_y = x_acc * np.sin(theta)

    return 180*np.arctan2(x_acc_in_y+gravity_y+linear_acc_in_y, x_acc_in_x+gravity_x) /np.pi

def discrete_state(theta, custom_theta, theta_dot):
    if theta < -12 or theta > 12: # 24
        return 256

    sign = 0
    if custom_theta < 0:
        custom_theta = -custom_theta
        sign = 1
    state = 2*(np.floor(8*custom_theta/180.0))

    if sign:
        state += 1

    state  = state * 16

    sign = 0
    if theta_dot < 0:
        theta_dot = -theta_dot
        sign = 1
    state += 2*(np.round(8*theta_dot/200))

    if sign:
        state += 1
    return np.int(state)

Q = np.ones((256+1, 2))
Q_prime = np.zeros((256+1, 2))

time_l = []
observations_old = np.zeros(4)

for i_episode in range(70):
    observation = env.reset()
    s_old = 0
    a_old = 0
    #s = 0
    done = 0
    for t in range(100):
        env.render()

        theta = 180*observation[2]/np.pi
        theta_dot = 180*observation[3]/np.pi

        raw_acc_angle = accelermeter_mode(theta, observation[3], observations_old[3], observation[1], observations_old[1])

        #print(theta, theta_dot, discrete_state(theta, theta_dot))

        observations_old = observation

        #action = env.action_space.sample()
        #print(action)
    
        s = discrete_state(theta, theta, theta_dot)
        #print("raw", theta_dot, raw_acc_angle, s)

        if s == 256:
            reward = -1
        else:
            reward = 0

        if Q[s, 0] > Q[s, 1]:
            action = 0
        else:
            action = 1

        Q_prime[s_old,a_old] = 0.99*(reward+0.99*Q[s,action]-Q[s_old,a_old])
        observation, reward_unused, done, info = env.step(action)
        print("action", action, "state", s)
        if s==256 or t==99:
            #print(Q)
            Q += Q_prime
            time_l.append(t+1)
            print("Episode finished after {} timesteps".format(t+1))
            break
        a_old = action
        s_old = s

print(time_l)
plt.plot(time_l)
plt.xlabel('episodes')
plt.ylabel('Num steps to failure')
plt.savefig('time.pdf')

env.close()
