class Joint:

    # class variable
    friction = 0.2

    def __init__(self, type, position = 0):
        self.type = type
        self.position = position
    
    def getPosition(self):
        return self.position
    
    @classmethod
    def updateFriction(cls, friction):
        cls.friction = friction

    @staticmethod
    def PoseTransformation(pose):
        pass

linear = Joint("linear", 1.57)

print(linear)
print(linear.type)
print(linear.getPosition)
print(linear.getPosition())
print(Joint.getPosition(linear))

# attributes of the instance
print(linear.__dict__)

Joint.updateFriction(0.5)

print(Joint.friction)