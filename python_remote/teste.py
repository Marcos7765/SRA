from coppeliasim_zmqremoteapi_client import *

# create a client to connect to zmqRemoteApi server:
# (creation arguments can specify different host/port,
# defaults are host='localhost', port=23000)
print("criando cliente")
client = RemoteAPIClient("10.255.255.254",verbose=True)

# get a remote object:
print("pedindo sim")
sim = client.require('sim')

print("pegando o chão")
# call API function:
h = sim.getObject('/Floor')
print(h)
