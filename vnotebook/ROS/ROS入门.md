# ROS入门
## ros程序包
一个程序包要想称为catkin程序包必须符合以下要求：

* 该程序包必须包含catkin compliant package.xml文件

这个package.xml文件提供有关程序包的元信息。

* 程序包必须包含一个catkin 版本的CMakeLists.txt文件，而Catkin metapackages中必须包含一个对CMakeList.txt文件的引用。

* 每个目录下只能有一个程序包。
这意味着在同一个目录下不能有嵌套的或者多个程序包存在。

```
cd ~/catkin_ws/src
```