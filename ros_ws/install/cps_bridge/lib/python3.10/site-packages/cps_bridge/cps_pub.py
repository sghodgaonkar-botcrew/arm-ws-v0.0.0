from coppeliasim_zmqremoteapi_client import *
from time import sleep

def myFunc(input1, input2):
    print('Hello', input1, input2)
    return 21


def main():
    print('Hi from cps_bridge.')

    # create a client to connect to zmqRemoteApi server:
    # (creation arguments can specify different host/port,
    # defaults are host='localhost', port=23000)
    client = RemoteAPIClient()

    # get a remote object:
    sim = client.require('sim')

    # call API function:
    h = sim.getObject('/Floor')
    print(h)

    # print(sim.getObjectHandle('/robot_base_respondable/connection_frame_visual'))
    print(sim.getObjectHandle('/robot_base_visual/Sphere_visual'))

    sim.startSimulation()

    sleep(5)

    sim.stopSimulation()
    

if __name__ == '__main__':
    main()
