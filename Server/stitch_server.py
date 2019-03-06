#
#   Stitching Server
#   Binds REP socket to tcp://*:5555
#

import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5555")

print("~~ STITCHING SERVER V1.0 ~~")
print("~~ I'm doing good work ! ~~")
print("~~~~~~~   <(-_-)>   ~~~~~~~")

while True:

    #  Wait for next request from client
    message = socket.recv()

    #  Do some 'work'.
    # time.sleep(1)

    #  Send reply back to client
    socket.send(message)
