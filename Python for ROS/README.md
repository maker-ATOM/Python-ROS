# Python for ROS

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#Basic-Python-OOP">Basic Python OOP</a></li>
    <li><a href="#Python for ROS">Python for ROS</a></li>
  </ol>
</details>

## Basic python OOP

- Data elements and functions associated to specific classes are called as attributes and methods.


General class description.
```
class joint:
    
    def __init__(self):
        pass
```

Instance of an class.
```
linear = joint()
```

```
print(linear)
<__main__.Joint object at 0x7effcedd3790>
```

- __init__ method is used to initialize the instances of the class. Similar to constructors.
- In python when ever a method is called by an instance of the class it is passed as an argument to the method. By convention we call it `self`.

- We do not need to pass the instance which is calling the method as an argument, it is taken care by the interpreter.
```
linear = joint()
```

Attributes set with the values provide by the arguments passed while creation of instance. 
```
    def __init__(self, type, position):
        self.type = type
        self.position = position

linear = Joint("linear", 1.57)
```

- If attributes of another instance are to be utilized then the instance needs to be passed a argument to the method.


Access attributes of the instances.
```
print(linear.type)
```

Usage of methods
```
print(linear.getPosition())
1.57
```

If parenthesis are not used for methods
```
print(linear.getPosition)
<bound method Joint.getPosition of <__main__.Joint object at 0x7f7abda67f40>>
``` 

If self is not passed as an argument in function definition
```
    def getPosition():
        return self.position

print(linear.getPosition())

Traceback (most recent call last):
  File "/home/aditya/Python-OOP/joint.py", line 18, in <module>
    print(linear.getPosition())
TypeError: Joint.getPosition() takes 0 positional arguments but 1 was given
```

We can run the methods from class as well but in this case we need to specify the instance for which the method is to be used.

```
print(Joint.getPosition(linear))
```

And if not passed as an argument.
```
Traceback (most recent call last):
  File "/home/aditya/Python-OOP/joint.py", line 19, in <module>
    print(Joint.getPosition())
TypeError: Joint.getPosition() missing 1 required positional argument: 'self'
```

Default arguments
```
    def __init__(self, type, position = 0):
        self.type = type
        self.position = position
```

Instance variables are associated with individual instances and differ for every other instance but class variables shared among all instances of the class.

- When we access a attribute it is first check if it is within the instance or not, if not then it checked if there exist any class variable of the same not or not, if not then base class (inherited class) is checked. 


Attributes of the instance
```
print(linear.__dict__)
```

To create a class method we simply need to use a decorator. Decorator are tool used to modify the behavior of a function or a class.
Convention to use class variable as cls.
The decorator needs to be used with every class method created
```
    @classmethod
    def updateFriction(cls, friction):
        cls.friction = friction
```

The class methods are usually called through class and the class is passed as an argument automatically.
```
Joint.updateFriction(0.5)
``` 

Regular methods automatically pass the instance as the first arguments, we call that self.
Class methods automatically pass the class as the first arguments, we call that cls.
Where static do not pass anything. They are used when the actions performed by the methods does not really depend if the a instance or the class is calling the function. Moreover if we do not use a instance or the class while performing operations then the method can be declared as static method.

```
    @staticmethod
    def PoseTransformation(pose):
        pass
```

## Python for ROS