<style type="text/css" rel="stylesheet">
a[href="#spoiler"] {
  text-decoration: none !important;
  cursor: default;
  margin-bottom: 10px;
  padding: 10px;
  background-color: #FFF8DC;
  border-left: 2px solid #ffeb8e;
  display: inline-block;
}
a[href="#spoiler"]::after {
  content: attr(title);
  color: #FFF8DC;
  padding: 0 0.5em;
}
a[href="#spoiler"]:hover::after,
a[href="#spoiler"]:active::after {
  cursor: auto;
  color: black;
  transition: color .5s ease-in-out;
}
</style>
# Week 2
Welcome to Week 2 of ROS training exercises! We'll be learning how to write **ROS Publishers** and **Subscribers**
with C++.

## Writing a ROS Subscriber and Publisher in C++
We've looked at how we can publish messages, now it's time to do it in C++. Before we do that though, let's
figure out how we can write a node in C++ first.

### Hello World with ROS and C++
Let's start by writing Hello World with ROS and C++. Starting off with a normal C++ Hello World in
[igvc_training_exercises/src/week2/main.cpp](../igvc_training_exercises/src/week2/main.cpp):
```C++
#include <iostream>

int main(int argc, char** argv)
{
  std::cout << "Hello World!" << std::endl;
}
```

We've already compiled this when compiling last week, but just as a recap, we can compile everything in the
catkin workspace by running
```bash
cd catkin_ws # cd to where your catkin workspace is located
catkin_make
```

Now, try running the executable with `rosrun`. The ROS package is called `igvc_training_exercises`, and the node is
called `week2`. You can refer back to [week 1](week1.md) on the details of the command if you forgot. Otherwise,
here's the [answer](#spoiler "rosrun igvc_training_exercises week2").

Verify that Hello World correctly prints out:
```
Hello World
```

Although we have a Hello World executable, this isn't a node yet. To make this a proper ROS node, we need to initialize
it.

Start by adding an *include* for the ROS headers at the top of the file. Don't worry about exactly what this does right now, as we'll be covering
this in general software training later.

```c++
#include <ros/ros.h> // Add this line

#include <iostream>
...
```

Next, add the following line above the `std::cout`:
```c++
int main(int argc, char** argv)
{
  ros::init(argc, argv, "week2"); // Add this line

  std::cout << "Hello World!" << std::endl;
}
```

`ros::init` initializes a ROS node, and the last argument in the function is the name of the node. To verify that it
works, save the file and recompile with `catkin_make`. You should still see `Hello World` being printed out. However, we
can't actually tell that it's working, because the program is exciting right after it prints `Hello World`. To make the
program wait, add this line after `std::cout`:

```c++
int main(iunt argc, char** argv)
{
  ros::init(argc, argv, "week2");

  std::cout << "Hello World!" << std::endl;
  ros::spin(); // Add this line
}
```

For now, don't worry about what `ros::spin()` does. All you need to know is that it stops the program from exiting until
you do Ctrl-C to stop it.

Compile the program again, and run it. Now, you should see that the program doesn't exit. Open up a new terminal window,
and type in `rosnode list`. You should see the `week2` node show up:
```
/rosout
/week2
```
