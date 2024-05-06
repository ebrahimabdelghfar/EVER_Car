import tkinter
import rospy
from std_msgs.msg import Float64
root = tkinter.Tk()
rospy.init_node("test_key")
publisher_cont = rospy.Publisher("/SteeringAngle",Float64,queue_size=0,latch=True)
publisher_cont_v = rospy.Publisher("/cmd_vel",Float64,queue_size=0)

steering = Float64()
velocity = Float64()
steer = 0.0
vel = 0.0

def key_handler(event:tkinter.Event):
    global steer , steering ,velocity ,vel
    if event.keycode == 111:
        print("up")
        if vel >= 20:
            vel = 20
        else:
            vel +=1.0
    elif event.keycode == 116:
        print("down")
        if vel <= 0:
            vel = 0
        else:
            vel -=1.0
    elif event.keycode == 114:
        if steer >= 28:
            steer = 28
        elif steer >=10 and steer <=12:
            steer = 15
        else:
            steer += 1.0
        print("right")
    elif event.keycode == 113:
        if steer <= -28:
            steer = -28
        else:
            steer -= 1.0
        print("left")
    steering.data = steer
    velocity.data = vel
    publisher_cont.publish(steering)
    publisher_cont_v.publish(velocity)

root.bind("<Key>", key_handler)

root.mainloop()


