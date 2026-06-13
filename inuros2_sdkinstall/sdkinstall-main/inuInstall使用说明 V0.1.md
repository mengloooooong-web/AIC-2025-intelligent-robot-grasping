### 1.把模组插入USB接口
  电脑的USB 规格比较多，有些是USB3.0， 有些是 USB2.0，最好都是接入USB3.0。如果使用USB HUB接入，最好是带供电功能的HUB。实际使用中，USB 口的稳定性对模组的影响比较大，当程序没有起来时，最好先尝试单模组方式，然后找到稳定的USB口，再尝试多模组。也可以找到一个稳定的USB口后，使用HUB扩展。
### 2.启动 inuInstall.sh
  
  sudo ./inuInstall.sh

如果接入了模组，应该看到以下画面，如果没有接入模组，那么程序会直接退出。

![](./pic/start.png)
<center>图一</center>

### 3.检测相机
在命令行中直接输入：
Input the selection: f

如果有相机则输出如下： （示例中插入了2个相机）

![](./pic/cameras.png)

### 4.安装单模组
**在命令行中直接输入：
Input the selection: 1**

等待一段时候后，再次回到命令界面，安装成功

### 5.安装多模组
**在命令行中直接输入：
Input the selection: 2**

在安装过程中，会提示：
camera number(Max=5):2
**最大支持 5 个相机，注意这里相机个数要和检测到的相机个数一致。**
Bind the camera with USB port: ( No - 0  Yes - 1 )
bind the USB port:
如果需要绑定端口，则输入1

**端口绑定的意义是：
相机绑定到USB 端口上，这样会加快一点点启动时间，建议不绑定。**

等待一段时候后，再次回到命令界面，安装成功

### 6.安装ROS1
**在命令行中直接输入：
Input the selection: 4**

首先要确保系统中已经安装了对应的ROS

在安装过程中，会提示：
camera number(Max=5):2
**最大支持 5 个相机，注意这里相机个数要和检测到的相机个数一致。**

等待一段时候后，再次回到命令界面，安装成功

### 7.启动ROS
**在命令行中直接输入：
Input the selection: 5**

可以看到ROS启动的打印，可以通过 rviz 查看图片

### 8.重启service
**在命令行中直接输入：
Input the selection: 6**

调试使用

### 9.卸载
**在命令行中直接输入：
Input the selection: u**

### 10.设置置信值
**在命令行中直接输入：
Input the selection: s**

请确认明白此值的意义，只能输入 1，3，5，7

值越大，结果越准确，但是输出的数据越少。