# PHOENIXEngine

# **概况**

![avatar](Documentation/markdownfile/img/mframe.png)

	开源，跨平台，室内机器人引擎

	open soruce cross platform system for robots

	从PHOENIX3D游戏引擎发展而来，用用C++/Lua 作为开发环境

	致力于游戏技术与机器人技术的融合

# **版本**

	版本：0.01

    功能完备的3D场景仿真渲染，UI组件等

    基于Windows和Liunx室内机器人自主定位建图，路径规划0.01版

	版本：0.02

	增强壁障算法，使其运行可以避开障碍

# **相关网址**

开源地址 码云:https://gitee.com/manykit/PHOENIXEngine

QQ交流群：336273110

网址: https://manykit.com

商城: https://shop194048616.taobao.com

MuBot视频:

Part1 结构安装：https://www.bilibili.com/video/av70786355
	
Part2 系统安装：https://www.bilibili.com/video/av70788879

Part3 展示：https://www.bilibili.com/video/av71408115

## **关于PHOENIXEngine**

	欢迎来到PHOENIXEngine，这是一个可以用来开发游戏，机器人，物联网项目的开发引擎。集成了HectorSlam，用来做为机器人室内导航。
	引擎已经支持Windows上，ARM，Linux，LinuxARM，iOS平台，使用简洁高效的Lua作为主要逻辑开发，使用C++/C作为底层构建语言。
	通过跨平台编译，可以将项目运行在所需要的平台上。
	借助Arduino，ESP8266/32等芯片，使用ManyKitArduino，引擎可以支持一般的物联设备开发。
	PHOENIXEngine只选择了最基本的两种语言C/C++和Lua。我们希望用户下载下路，编译即可运行。	

## **作者**
    
    引擎一直由 许多（ManyXu） 在开发维护，ManyXu从腾讯IEG，思前想后，多番权衡，辞职后，孤身来到了深圳华强北，开了一家小公司-深圳许多米科技，
    从事跨平台机器人软硬件的开发。今天将其开源出来，终于开始小规模测试。

##  **编译/Compile**

### *Windows*

使用visual studio打开PHOENIXEngine/PHOENIX.sln编译,

平台配置选择OpenGLRelease|Win32或OpenGLDebug|Win32

如不编译，直接运行PHOENIXEngine/Bin/AppPlayerOpenGL.exe
	
### **Linux Raspberry**

1.配置
	
	sudo sh ./linuxbuildconfigarm.sh

2.编译引擎库 libPX2Engine.so (注意这里不需要sudo):
	
	<!------如直接，运行已编译好的文件，跳过该步骤>

	sh ./linuxbuildarm.sh

3.编译Slam2D:

	<!------如直接，运行已编译好的文件，跳过该步骤>

	进入目录PHOENIX/PX2Engine/AI/Slam2D 运行

	sh linuxbuildarm.sh

4.安装引擎库:

	sudo sh ./linuxbuildinstallarm.sh

5.执行

	<!------如直接，运行已编译好的文件，运行命令，增加运行权限
		sudo chmod 777 ./AppPlayer.ReleaseARMDynamic
	>
	
	在PHEONIXEngine/PHOENIX/Bin目录执行 ./AppPlayer.ReleaseARMDynamic
	
	树莓派默认OpenGL驱动没有激活，可以直接执行 ./AppPlayer.ReleaseARMDynamic window=0 以命令行运行程序

	若需要启动OpenGL驱动硬件加速，需使用sudo raspi-config进行修改，修改后，需使用hdmi显示屏幕，插针的屏幕将无法显示。

6.系统上电，自动启动配置[查看wiki](https://gitee.com/manykit/PHOENIXEngine/wikis/Home)系统启动时自动运行程序或脚本
	
### **Linux**

当前目录为PHOENIXEngine/PHOENIX

1.配置
	
	sudo sh ./linuxbuildconfig.sh

2.编译引擎库libPX2Engine.so(注意这里不需要sudo):

	sh ./linuxbuild.sh

3.编译Slam2D:

	进入目录PHOENIX/PX2Engine/AI/Slam2D 运行

	sh linuxbuild.sh


4.安装引擎库:

	sudo sh ./linuxbuildinstall.sh

5.运行最终的可执行程序
	
	在PHOENIXEngine/PHOENIX/Bin目录执行 ./AppPlayer.ReleaseDynamic
	
6. 注意事项

	Linux中的串口需要一般在sudo模式下才能或许，执行脚本sudo sh linuxserialsudo.sh 可以将串口模式变为正常模式也可以读取（需要重新插拔usb线）

	如果出现例似如下错误：Syntax error: "(" unexpected。首先可以用命令ls -l /bin/sh 看看当前默认的是什么shell，Ubuntu一般默认为bash

    修改默认的shell，可以采用命令sudo dpkg-reconfigure dash。选择否。

### **Android**
	
	待续

### **IOS**

	使用XCode打开项目PHOENIX/Phoenix.xcworkspace编译项目，目前该配置比较老，未更新
	
## **更多内容**

	https://gitee.com/manykit/PHOENIXEngine/wikis/Home

## **版权信息/Licence**

	ManyKit开源协议：
	PHOENIXEngine是一个开源开放的引擎，拥有免费版本和商业版本，开源版本只需要遵循较少的规则就可以免费用于商业开发，商业版本包含相关的技术服务。协议声明如下：
	1.任何人，组织都可以自由修改引擎，免费应用于商业开发。
	2.使用PHOENIXEngine免费版本，必须在醒目位置标注PHOENIXEngine的商标，并且在软件中出现PHOENIXEngine开源协议，即本协议。
	3.用户开发的扩展包，可以选择开源或者闭源，对自己开发的开发包享有完全的自主控制权利。
	4.许多米科技保留对PHONEIXEngine，PHOENIXFrame，MuBot的商标权。
	5.许多米科技目前保留对引擎的教育培训领域的完全使用权，第三方如想要使用PHOENIXEngine作为培训教育，许多米可以进行相关培训，获得相关认证。
	6.使用PHOENIXEngine开发的商业产品，许多米科技可以进行销售。
	7.许多米科技目前保留对引擎技术社区，包含项目交流社区的独家开发使用权。
	8.任何捐赠第三方必须首先提出对引擎修改意见和技术支持的具体需求。
