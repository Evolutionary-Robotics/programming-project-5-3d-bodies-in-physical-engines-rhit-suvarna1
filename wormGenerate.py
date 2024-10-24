import pyrosim.pyrosim as ps


# Global parameters
l = 1 # length
w = 1 # width 
h = 1 # height 

x = 0
y = 0
z = 0.5

#Created static example 2 boxes
def Create_World():
    ps.Start_SDF("box.sdf")
    ps.Send_Cube(name="Box",pos=[x,y,z],size=[l,w,h])
    ps.End()
    ps.Start_SDF("box2.sdf")
    ps.Send_Cube(name="Box",pos=[1,0,0.1],size=[l,w,h])
    ps.End()

def generateWorm():
    ps.Start_URDF("worm.urdf")
    ps.Send_Cube(name = "Head", pos=[x,y,z], size = [l,w,h])
    ps.Send_Joint(name = "Head_Joint1", parent="Head", child="Joint1", type="revolute", position=[0.5,0,0])
    ps.Send_Cube(name = "Joint1",pos=[0.5,0.0,0.5], size = [l,w,h])
    ps.Send_Joint(name = "Joint1_Joint2", parent="Joint1", child="Joint2", type="revolute", position=[1,0,1])
    ps.Send_Cube(name = "Joint2",pos=[0.5,0.0,-0.5], size = [l,w,h])
    ps.Send_Joint(name = "Joint2_Joint3", parent="Joint2", child="Joint3", type="revolute", position=[1,0,0])
    ps.Send_Cube(name = "Joint3",pos=[0.5,0.0,-0.5], size = [l,w,h])
    ps.Send_Joint(name = "Joint3_Joint4", parent="Joint3", child="Joint4", type="revolute", position=[1,0,0])
    ps.Send_Cube(name = "Joint4",pos=[0.5,0.0,-0.5], size = [l,w,h])
    ps.Send_Joint(name = "Joint4_Joint5", parent="Joint4", child="Tail", type="revolute", position=[1,0,0])
    ps.Send_Cube(name = "Tail",pos=[0.5,0.0,-0.5], size = [l,w,h])
    ps.End()


generateWorm()
# Create_World()