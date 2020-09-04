#!/usr/bin/env python3

import numpy as np
import smarties as rl
import sys

from model import ABFs

class Status:
    Running = 0
    Failed = 1
    Success = 2

class Environment:
    def __init__(self,
                 bmb: list=[1, 1],
                 cmb: list=[1, 2],
                 dt: float=1):
        self.bmb = bmb
        self.cmb = cmb
        self.step = 0
        self.dt = dt

    def reset(self):
        x0 = np.random.uniform(-50, 50, len(self.bmb))
        self.abfs = ABFs(x0, self.bmb, self.cmb)
        self.step = 0
        self.w = 0
        self.prev_distances = np.abs(self.abfs.x)

    def isOver(self): # is episode over
        return self.step >= 500

    def isSuccess(self): # has reached the target
        rt = 1
        d = np.abs(self.abfs.x)
        return np.max(d) < rt

    def advance(self, action):
        self.w = action[0]
        self.abfs.advance(self.w, self.dt)
        self.step += 1

        if self.isOver():
            return Status.Failed
        elif self.isSuccess():
            return Status.Success
        else:
            return Status.Running

    def getState(self):
        state = np.copy(self.abfs.x)
        return state

    def getReward(self):
        r = - self.dt

        distances = np.abs(self.abfs.x)
        r += np.sum(self.prev_distances - distances)
        self.prev_distances = distances

        if self.isSuccess():
            r += 10

        return r

def app_main(comm):
    env = Environment()
    env.reset()
    nstates = len(env.getState())
    comm.setStateActionDims(nstates, 1)
    comm.setActionScales([-5.0], [5.0], areBounds=True)
    comm.setStateObservable([True] * nstates)

    while 1: #train loop, each new episode starts here
        env.reset()
        comm.sendInitState(env.getState());

        while 1: #simulation loop
            action = comm.recvAction();

            status = env.advance(action);

            state  = env.getState();
            reward = env.getReward();

            if status == Status.Failed:
                comm.sendLastState(state, reward);
                break
            elif status == Status.Success:
                comm.sendTermState(state, reward);
                break
            else:
                assert status == Status.Running
                comm.sendState(state, reward);


if __name__ == '__main__':
    e = rl.Engine(sys.argv)
    if e.parse():
        exit(1)
    e.run(app_main)
