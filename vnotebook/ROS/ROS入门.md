# ROS入门

## ROS工作空间
### catkin工作空间
* 创建catkin空间
```
mkdir -p ./catkin_ws(your space name)/src
cd ./catkin_ws  # 进入工作空间root目录
catkin_make  # 编译空间
```
* `catkin_make `命令在**catkin**工作空间中是一个非常方便的工具。如果你查看一下当前目录应该能看到`build`和`devel`这两个文件夹。
build 目录是build space的默认所在位置，同时cmake 和 make也是在这里被调用来配置并编译你的程序包。devel 目录是devel space的默认所在位置, 同时也是在你安装程序包之前存放可执行文件和库文件的地方
*  在`devel`文件夹里面你可以看到几个setup.*sh文件。source这些文件中的任何一个都可以将当前工作空间设置在ROS工作环境的最顶层，想了解更多请参考catkin文档。接下来首先source一下新生成的setup.*sh文件：

```
source devel/setup.bash

```
* 要想保证工作空间已配置正确需确保ROS_PACKAGE_PATH环境变量包含你的工作空间目录，采用以下命令查看：

```
echo $ROS_PACKAGE_PATH
/home/<youruser>/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks
```

-----
## ros程序包
一个程序包要想称为catkin程序包必须符合以下要求：

* 该程序包必须包含`catkin compliant package.xml`文件

这个package.xml文件提供有关程序包的元信息。

* 程序包必须包含一个catkin 版本的`CMakeLists.txt`文件，而`Catkin metapackages`中必须包含一个对CMakeList.txt文件的引用。

* 每个目录下只能有一个程序包。
这意味着在同一个目录下不能有嵌套的或者多个程序包存在。

```
my_packege
    cMakeList.txt
    package.xml
```
### 创建catkin程序包
* 在catkin工作空间中的src目录下
现在使用catkin_create_pkg命令来创建一个名为'beginner_tutorials'的新程序包，这个程序包依赖于std_msgs、roscpp和rospy：
```
 catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

------
### 查看包的依赖关系
####  直接依赖
```
$ rospack depend1 beginnner_tutorials
```

```
std_msgs
rospy
roscpp
```
* rospack列出了在运行catkin_create_pkg命令时作为参数的依赖包，这些依赖包随后保存在`beginner_tutorials /package.xml`文件中。

```
<package>
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
```
#### 间接依赖
* 在很多情况中，一个依赖包还会有它自己的依赖包，比如，rospy还有其它依赖包

------
### 自定义程序包
* 自定义`package.xml`

```
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorials</name>
  <version>0.0.0</version>
  <description>The beginner_tutorials package</description>  ## 首先更新描述标签

### 维护者标签
  <!-- One maintainer tag required, multiple allowed, one person per tag -->
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="bornchow@todo.todo">bornchow</maintainer>

### 许可标签
  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>BSD</license>


  <!-- Url tags are optional, but multiple are allowed, one per tag -->
  <!-- Optional attribute type can be: website, bugtracker, or repository -->
  <!-- Example: -->
  <!-- <url type="website">http://wiki.ros.org/beginner_tutorials</url> -->


  <!-- Author tags are optional, multiple are allowed, one per tag -->
  <!-- Authors do not have to be maintainers, but could be -->
  <!-- Example: -->
  <!-- <author email="jane.doe@example.com">Jane Doe</author> -->

### 依赖项标签
### 接下来的标签用来描述程序包的各种依赖项，这些依赖项分为build_depend、buildtool_depend、run_depend、test_depend
  <!-- The *depend tags are used to specify dependencies -->
  <!-- Dependencies can be catkin packages or system dependencies -->
  <!-- Examples: -->
  <!-- Use depend as a shortcut for packages that are both build and exec dependencies -->
  <!--   <depend>roscpp</depend> -->
  <!--   Note that this is equivalent to the following: -->
  <!--   <build_depend>roscpp</build_depend> -->
  <!--   <exec_depend>roscpp</exec_depend> -->
  <!-- Use build_depend for packages you need at compile time: -->
  <!--   <build_depend>message_generation</build_depend> -->
  <!-- Use build_export_depend for packages you need in order to build against this package: -->
  <!--   <build_export_depend>message_generation</build_export_depend> -->
  <!-- Use buildtool_depend for build tool packages: -->
  <!--   <buildtool_depend>catkin</buildtool_depend> -->
  <!-- Use exec_depend for packages you need at runtime: -->
  <!--   <exec_depend>message_runtime</exec_depend> -->
  <!-- Use test_depend for packages you need only for testing: -->
  <!--   <test_depend>gtest</test_depend> -->
  <!-- Use doc_depend for packages you need only for building documentation: -->
  <!--   <doc_depend>doxygen</doc_depend> -->
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
### run_depend
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>


  <!-- The export tag contains other, unspecified, tags -->
  <export>
    <!-- Other tools can request additional information be placed here -->

  </export>
</package>
```
------
## 编译ros程序包
### 使用`catkin_make`
catkin_make 是一个命令行工具，它简化了catkin的标准工作流程。你可以认为catkin_make是在CMake标准工作流程中依次调用了cmake 和 make。
```
catkin_make [make-targets] [-DCMAKE-VARIABLES=...]
```
* 每个CMake工程在编译时都会执行这个操作过程。相反，多个catkin项目可以放在工作空间中一起编译，工作流程如下

```
# In a catkin workspace
$ catkin_make
$ catkin_make install  # (可选)
```
* 如果你的源代码不在默认工作空间中（~/catkin_ws/src),比如说存放在了my_src中，那么你可以这样来使用catkin_make:

```
# In a catkin workspace
$ catkin_make --source my_src
$ catkin_make install --source my_src  # (optionally)
```

------
## 理解ROS节点


* Nodes:节点,一个节点即为一个可执行文件，它可以通过ROS与其它节点进行通信。
* Messages:消息，消息是一种ROS数据类型，用于订阅或发布到一个话题。
* Topics:话题,节点可以发布消息到话题，也可以订阅话题以接收消息。
* Master:节点管理器，ROS名称服务 (比如帮助节点找到彼此)。
* rosout: ROS中相当于stdout/stderr。
* roscore: 主机+ rosout + 参数服务器 (参数服务器会在后面介绍)。

### 节点
* 一个节点其实只不过是ROS程序包中的一个可执行文件。ROS节点可以使用ROS客户库与其他节点通信。节点可以发布或接收一个话题。节点也可以提供或使用某种服务。

* ROS客户端库允许使用不同编程语言编写的节点之间互相通信:

rospy = python 客户端库
roscpp = c++ 客户端库

### roscore
* roscore 是你在运行所有ROS程序前首先要运行的命令
```
roscore
```

### rosnode
rosnode 显示当前运行的ROS节点信息。rosnode list 指令列出活跃的节点:

### rosrun
rosrun 允许你使用包名直接运行一个包内的节点(而不需要知道这个包的路径)。