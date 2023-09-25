# Python for ROS

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#Python-Tricks">Python Tricks</a></li>
    <li><a href="#Strings">Strings</a></li>
    <li><a href="#Numerics">Numerics</a></li>
    <li><a href="#data-structures">Data Structures</a></li>
    <li><a href="#Control-Logic">Control logic</a></li>
    <li><a href="#Tests">Tests</a></li>
    <li><a href="#Exceptions">Exceptions</a></li>
    <li><a href="#Basic-Python-OOP">Basic Python OOP</a></li>
    <li><a href="#Package-Module-Class">Package vs Module vs Class</a></li>
    <li><a href="#Python for ROS">Python for ROS</a></li>
  </ol>
</details>

<!-- - Tick: ✓ or &#x2713; or &#10003;
- Cross: ✗ or &#x2717; or &#10007; -->

<!-- ```
- Tick: ✓ or &#x2713; or &#10003;
- Cross: ✗ or &#x2717; or &#10007;
``` -->



beginner 9
pip
vm
var scope
sclice
comrehension
sort
string formatting
os mod
automate
rand
csv
regex
try execopt
duck
fstring
generators
decorators
nametuple
oop
resources
str vs repr
if name = main
unit testing
else on loop
set
iterables



## Python tricks

```python
dir(object) => methods available with the object
help(class) => description of class
help(method) => description of method
```

## Strings

```python
✓ data = 'Robot Structure'
✗ data = 'Robot's structure'
✓ data = 'Robot\'s structure' => escape character
✓ data = "Robot's structure"
✓ data = """Robot structure
            and its world""" => multi line string, works with single and double quotes

len(data) => number of characters
data[index] => access individual characters
data[0:3] => slicing
✓ new = data + 'lidar' => concatenation

Escape sequences:
\t => tb space
\n => new line
\\ => backslash
\' =>single quotes

Methods:
data.upper() => coverts to uppercase
data.lower() => coverts to lowercase
data.isupper()
data.islower()
data.isspace()
data.isalnum()
data.aplha()
data.isTitle()
data.replace()

Formatting:
greeting = 'Hello'
name = 'Aditya'


✓ message = greeting + ', ' + name + '.Welcome!'
✓ message = '{}, {}. Welcome!'.format(greeting, name) => placeholders
✓ message = f'{greeting}, {name}. Welcome!' => fstrings
```

## Numerics

```python
Arithmetic Operators:
Addition:       3 + 2
Subtraction:    3 - 2
Multiplication: 3 * 2
Division:       3 / 2
Floor Division: 3 // 2
Exponent:       3 ** 2
Modulus:        3 % 2

Comparisons:
Equal:            3 == 2
Not Equal:        3 != 2
Greater Than:     3 > 2
Less Than:        3 < 2
Greater or Equal: 3 >= 2
Less or Equal:    3 <= 2

abs()
round()
int() => typecast
```

## Data Structures

```python
list[]
tuple()
set{}

list.append()
list.extend()
list.insert()
list.remove()
list.pop()
list.sort()
list.reverse()
list.index()
enumerate(list)

✓ list[n-1] == list[-1]

list[start(inclusive): end(exclusive): step] => slicing
list[:] => shallow copy
list1.insert(n, list2) != list1.extend(list2)
list_str = ', '.join(list) => conversion of list to string
list = list_str.split(" ") => split on space

set.intersection()
set.difference()
set.union()

✗ set = {} => creates a dict
✓ set = set()

dict.update({dict2})
del dict.[key]
student.pop[key]
dict.values()
dict.key()

dict[key] => throws error if key is not present
dict.get(key) => returns if key is not present

val = dict[key] => returns the val of the key
dict[key] = val => updated the val or adds a new pair

for key in dict:
    pass

for key, value in dict.items():
    pass
```

## Control Logic

```python
Equality: ==
Identity: is (checks id)

False Values:
    False
    None
    Zero of any numeric type
    Any empty sequence. For example, '', (), [].
    Any empty mapping. For example, {}.

for item in list:
    print(item)

continue => loop to next iteration without executing below statements
break => break the loop
```

## Tests

Tests are required to check the code if there are any changes made to the code. There might be a situation where the function works correctly but the output generated brakes the other section of code, unit test are helpful in this case.

Print statements are hard to monitor and also automate, so unit tests are used.

When a class is build to perform unit test a method is associated with the function within the `to be tested` source code. The method name should start with `test_`

Each test method is considered as a single test.

```python
    def test_add(self):
        self.assertEqual(calc.add(10, 5), 15)
        self.assertEqual(calc.add(11, 5), 16)
        self.assertEqual(calc.add(-1, 1), 0)
```

So this will return,

```python
.
----------------------------------------------------------------------
Ran 1 test in 0.000s

OK
```

If a particular operation is repeated in all test methods we can use `setUp()` and `tearDown` methods to simplify this process. setUp method is runned before every single test methods and tearDown is executed after every single test method.

Test method should not dependent of each other because the oder in which they executed is not a defined one.

Since setUp and tearDown run for every test method we can also ave a setUp and tearDown method for class `setUpClass` `tearDownClass` which are executed once before and after all test method.

Mocking is used when something other our function is dependent has failed but the function is supposed to work correctly.  

## Exceptions

When a particular error (run-time) occurs when we handle it by performing dedicated task, if the error is not handled then execution of the script may be halted to terminated.

Moreover our script also can raise exceptions if necessary.

## Basic python OOP

Data elements and functions associated to specific classes are called as attributes and methods.

We can also say methods are functions associated to objects.
  
Every value is an object. Whether it be a dictionary, a list, or even an integer, they are all objects. Programs manipulate those objects either by performing computation with them or by asking them to perform methods. Every object has state that is the value it holds and a collection of methods that it can perform.  The state of an object represents those things that the object knows about itself. The state is stored in instance variables.

General class description.
```python
class joint:
    
    def __init__(self):
        pass
```

Instance of an class.
```python
linear = joint()
```

```python
print(linear)
<__main__.Joint object at 0x7effcedd3790>
```

- __init__ method is used to initialize the instances of the class. Similar to constructors.
- In python when ever a method is called by an instance of the class it is passed as an argument to the method. By convention we call it `self`.

- We do not need to pass the instance which is calling the method as an argument, it is taken care by the interpreter.
```python
linear = joint()
```

Attributes set with the values provide by the arguments passed while creation of instance. 
```python
    def __init__(self, type, position):
        self.type = type
        self.position = position

linear = Joint("linear", 1.57)
```

- If attributes of another instance are to be utilized then the instance needs to be passed a argument to the method.


Access attributes of the instances.
```python
print(linear.type)
```

Usage of methods
```python
print(linear.getPosition())
1.57
```

If parenthesis are not used for methods
```python
print(linear.getPosition)
<bound method Joint.getPosition of <__main__.Joint object at 0x7f7abda67f40>>
``` 

If self is not passed as an argument in function definition
```python
    def getPosition():
        return self.position

print(linear.getPosition())

Traceback (most recent call last):
  File "/home/aditya/Python-OOP/joint.py", line 18, in <module>
    print(linear.getPosition())
TypeError: Joint.getPosition() takes 0 positional arguments but 1 was given
```

We can run the methods from class as well but in this case we need to specify the instance for which the method is to be used.

```python
print(Joint.getPosition(linear))
```

And if not passed as an argument.
```python
Traceback (most recent call last):
  File "/home/aditya/Python-OOP/joint.py", line 19, in <module>
    print(Joint.getPosition())
TypeError: Joint.getPosition() missing 1 required positional argument: 'self'
```

Default arguments
```python
    def __init__(self, type, position = 0):
        self.type = type
        self.position = position
```

Instance variables are associated with individual instances and differ for every other instance but class variables shared among all instances of the class.

- When we access a attribute it is first check if it is within the instance or not, if not then it checked if there exist any class variable of the same not or not, if not then base class (inherited class) is checked. 


Attributes of the instance
```python
print(linear.__dict__)
```

Custom print statement associated to each instance can be generated using __str__ method
```python
    def __str__():
        return <custom_string>

print(<instance_of_the_class>)
```

To create a class method we simply need to use a decorator. Decorator are tool used to modify the behavior of a function or a class.
Convention to use class variable as cls.
The decorator needs to be used with every class method created
```python
    @classmethod
    def updateFriction(cls, friction):
        cls.friction = friction
```

The class methods are usually called through class and the class is passed as an argument automatically.
```python
Joint.updateFriction(0.5)
``` 

Regular methods automatically pass the instance as the first arguments, we call that self.
Class methods automatically pass the class as the first arguments, we call that cls.
Where static do not pass anything. They are used when the actions performed by the methods does not really depend if the a instance or the class is calling the function. Moreover if we do not use a instance or the class while performing operations then the method can be declared as static method.

```python
    @staticmethod
    def PoseTransformation(pose):
        pass
```

```python
def func(*args, **kwargs): => accepts arbitrary number of parameters
    print(args) => all positional arguments
    print(kwargs) => all key word arguments

func('math','art', name='john', age=23)
```

## Package Module Class

All these three help in structuring and organizing code.

**Class**

A class is a blueprint for creating objects (instances).
It defines a set of attributes (variables) and methods (functions) that the objects created from the class will have.
Classes are used for Object-Oriented Programming (OOP) and allow us to model real-world entities and their behavior.
We can create instances of a class to work with its attributes and methods.

```python
class Dog:
    def __init__(self, name):
        self.name = name

    def bark(self):
        print(f"{self.name} says Woof!")

my_dog = Dog("Buddy")
my_dog.bark()  # Calls the 'bark' method of the 'Dog' class for 'my_dog' instance.
```

**Module**

A module is a single Python file that contains variables, functions, and classes.
It is used to organize related code into separate files, making it easier to manage and reuse code.
Modules can be imported and used in other Python scripts to access their contents.

```python
# This is a module named 'my_module.py'
def add(a, b):
    return a + b

def subtract(a, b):
    return a - b

# In another script, you can import and use functions from 'my_module.py'
import my_module
result = my_module.add(3, 2)
```

when a module is imported is it executed hence any print or log statement will be executed. Moreover if want a particular method to be imported we can,'

```python
from my_module import add
```

```python
print(sys.path) => returns the paths python locks for while finding the module
```

To add a custom path of module we have to update the env variable with the custom path

**Package**

A package is a directory that contains a collection of Python modules and a special __init__.py file (which can be empty).
It is used to organize related modules into a hierarchical directory structure.
Packages allow you to group related functionality and create namespaces to avoid naming conflicts.
To use modules within a package, you import them using dot notation.

```python
my_package/
├── __init__.py
├── module1.py
└── module2.py
```
To use a class within the module module1.py of the package my_package,

```python
class my_package.module1 import class_name

my_instance = my_class(attribute)
```