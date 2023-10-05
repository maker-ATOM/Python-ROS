# Python for ROS

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#Data-Structures">Data Structures</a></li>
    <li><a href="#Control-Logic">Control logic</a></li>
    <li><a href="#Tests">Tests</a></li>
    <li><a href="#Exceptions">Exceptions</a></li>
    <li><a href="#OOP">OOP</a></li>
    <li><a href="#Package-Module-Class">Package vs Module vs Class</a></li>
    <li><a href="#PIP">PIP</a></li>
    <li><a href="#OS">OS</a></li>
    <li><a href="#Thread">Thread</a></li>
    <li><a href="#MutliProcessing">MutliProcessing</a></li>
    <li><a href="#GPT">GPT</a></li>
  </ol>
</details>

<!-- [Resource](https://www.youtube.com/playlist?list=PL-osiE80TeTt2d9bfVyTiXJA-UTHn6WwU) -->


**Variable scope**

```python
First it is checks if the variable is present in the local scope rf not
then checks if it is available in enclosing scope and 
if not then in global or else throws a error.

Local => Defined within function
Enclosing => Function within a function 
Global => Top level of module
Build-in => Predefined within python

locals can access global stuff
but globals cannot access locals

nonlocal => allows to use variable from enclosing function
```

**Lamda Function**

```python

```

**Duck Typing (Pythonic)**

```python
An object if cn walk like a duck and quack like a duck then it is a duck,
that is we do not care what type of object we are working with 
if the object can do what we ask it to do.

```

**Look before you leap (Non-Pythonic)**

```python
Check if the attribute exists before calling.
```

**Easier to ask for forgiveness than permission (Pythonic)**

```python
Do not check before calling, call it, if it does exist then handle it using Exception.
```

**If _name__ == '__main__'**

```python
To check a file is executed directly by python is being imported.
```

**Generators**

```python
Do not hold the complete value in memory. Generates the result when the called one by one. 
Does not performs the operation until the result is being used.
```

**Decorators**

```python
Python decorators provide a way to modify or extend the behavior of functions or 
methods without modifying the underlying function's code. 
Decorators are functions that wrap other functions to extend their behavior transparently.
```

**Json**

    <li><a href="#Recursion">Recursion</a></li>
```python
Data format to store data and also to store config data.
```

**Regex**

```python
Help to match specific pattern in text.
```

    <li><a href="#Recursion">Recursion</a></li>
**Iterable**

```python
Something that can be looped over.
If something is iterable then a special method __iter__ is associated with the object

list is not a iterator but is iterable that is, when the __iter__ method is executed on list it returns a iterator, 
iterator is object with an state so that it remember its state during iteration. 
The next value of iterator is received through next method.

we can also add these method to our own class and make the objects iterables.
```

**Context manger**

```python
Manage resource efficiently.
```

**Unpacking**

```python
a, b = (1, 2)
```

**Recursion**

```python
A recursive function is a function that calls itself.
Used to solve problems that can be broken down into smaller more manageable sub-problems.

All recursive function have a base case which acts as termination for the recursion.
```

## Data Structures

```python
p, q = q, p => valid swapping without need of temp variable.

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
    <li><a href="#Recursion">Recursion</a></li>

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

    <li><a href="#Recursion">Recursion</a></li>
str()
repr()

The goal of __repr__ is to be unambiguous (returns a executable data format)
The goal of __str__ is to be readable

```

```python
    <li><a href="#Recursion">Recursion</a></li>

abs()
round()
int() => typecast

1000000000000 is same as 1_000_000_000_000 => for readability

```

```python

    <li><a href="#Recursion">Recursion</a></li>
list[]

list.append()
list.extend()
list.insert()
list.remove()
list.pop()
list.sort()
sorted(list, key) => key is used when list elements are object which have attributes associated.
list.reverse()
list.index()

for index, element in enumerate(list):
    print(index, element)

✓ list[n-1] == list[-1]
list[start(inclusive): end(exclusive): step] => slicing
list[:] => shallow copy

list1.insert(n, list2) != list1.extend(list2)
list_str = ', '.join(list) => conversion of list to string
list = list_str.split(" ") => split on space

List Comprehension

nums = [1,2,3,4,5,6,7,8,9,10]

# I want 'n' for each 'n' in nums
my_list = []
for n in nums:
  my_list.append(n)

same as

my_list = [n for n in nums]

# I want 'n' for each 'n' in nums if 'n' is even
my_list = []
for n in nums:
  if n%2 == 0:
    my_list.append(n)

same as 

my_list = [n for n in nums if n%2 ==0

# I want a (letter, num) pair for each letter in 'abcd' and each number in '0123'
my_list = []
for letter in 'abcd':
  for num in range(4):
    my_list.append((letter,num))]

same as

mu_list = [(letter, num) for letter in 'abcd' for num in range(4)]

zip(list1, list2, ...) => creates a tuple of items

for element1, element2 in zip(list1, list2): => to use two list at a time 
    print(element1, element2)

visited = [False] * 5 <same as > visited = [False for _ in range(5)]

```

```python

set{} => no duplicates

set.intersection()
set.difference()
set.union()

set.add(int)
set.update(list or set)

set.remove() => throws a error if we try remove a value that does not exist
set.discard() => does not throw error

✗ set = {} => creates a dict
✓ set = set()

```

```python

dict.update({dict2})
del dict.[key]
student.pop[key]
dict.values()
dict.key()

sorted(dict) => sorts key

dict[key] => throws error if key is not present
dict.get(key) => returns if key is not present

val = dict[key] => returns the val of the key
dict[key] = val => updated the val or adds a new pair

for key in dict:
    pass

for key, value in dict.items():
    pass

```

```python

color =(55,255,255) => regular tuple

Color = namedtuple('Color', ['red', 'green', 'blue']) => namedtuple
color = Color(55,155,255)
another_color = Color(0,77,145)
print(color.red)

Mid way between tuple and dict
---

prefer secrets module instead of random module
```

Double Ended Queue

```python
from collections import deque 
container = deque()

container.append(num)
container.appendleft(num)
container.pop()
container.popleft()
container.len()
container.index(num, beg, end):- This function returns the first index of num, starting searching from beg till end index.
container.insert(i, a) :- This function inserts a at index(i).
container.remove(num):- This function removes the first occurrence num.
container.count(num):- This function counts the number of occurrences of ele.

Supports positive and negative indexing
```

```python
stack = []
stack.append(num)
stack.pop()

from collections import deque
stack = deque() 
stack.append(num)
stack.pop()
```

## Control Logic

```python
if a <= b <= c: => valid range checker
    pass

condition = False
x = 1 if condition else 0 => ternary conditions

Equality: ==
Identity: is (checks id(memory address of the object))  

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

Else clause loop

for i in list:
    print i
else:
    print("Else Clause")

Executes when all the iterations of the for are executed. Used in conjugation with `break`.

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

```python
try:
    pass
except <exception>:
    pass
finally:
    pass
```

## OOP

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
dir(object) => methods available with the object
help(class) => description of class
help(method) => description of method
```

```python
print(linear)
<__main__.Joint object at 0x7effcedd3790>
```

- __init__ method is used to initialize the instances of the class. Similar to constructors.
- In python when ever a method is called by an instance of the class it is passed as an argument to the method. 
  By convention we call it `self`.

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

## PIP

Python Package Manager

```python
pip help

pip search <package_name>
pip install <package_name>
pip list

pip freeze > requirements.txt => store the package list in a file
pip install -r requirements.txt => install all packages within the list

pipenc = pip + virtual env
```

## OS

```python
os.getcwd()
os.chdir(path)
os.listdir()
os.mkdir()
os.makedirs()

os.stat(file)
os.walk()
os.environ.get()
os.path.join()
```

## Threads

```python
Running different task concurrently.

t1 = threading.thread(target=<function_to_be_executed>) => creates a object 
t1.start() => starts the process
```

## MultiProcessing

```python
Task running in parallel
```

## GPT

