# Unmanned-aircraft-control-system-for-Boeing747
本工作以大型民用客机波音747-100为研究对象，建立了自动飞行控制系统的整体数学模型，并对飞行过程进行综合数字仿真。  
This work takes the large civil airliner Boeing 747-100 as the research object, establishes an overall mathematical model of the automatic flight control system, and conducts a comprehensive digital simulation of the flight process.  

本工作为本科毕设项目，以大型民用客机为研究对象，基于飞行运行学与动力学对飞机仿真模型和飞行状态进行模拟，建立了自动飞行控制系统的整体数学模型。在此基础上，利用经典控制方法设计飞机在各类飞行工作模态的控制律，并对整个空域的飞行过程进行综合数字仿真。工作中具体完成了以下任务：  
* 1) 建立六自由度的无人机运动模型，并通过简化得到横向和纵向运动学与动力学的传递函数模型；  
* 2) 采用连续闭环控制的方法设计了飞机型无人机的自动控制系统，包括滚转姿态环、航迹保持、侧滑保持、俯仰姿态控制等多个控制回路；  
* 3）在Simulink程序中建立了波音747-100客机的自动驾驶仪模型，调整各控制参数；  
* 4) 模拟飞机的横向和纵向自动驾驶仪，以及飞机飞行的一般过程，并由此得出结论。  
经过仿真验证，结果表明：采用以上方法设计的无人机自动控制系统能够准确地达到预期的控制指标，能较好地保证飞机飞行过程中的安全性和稳定性。

版本：Matlab2020a
主要参考书目：Beard R. W., McLain T. W. Small Unmanned Aircraft: Theory and Practice, Princeton University Press, 2012. 主要程序按照该书2-6章节设计，飞机参数采用NASA的Boeing747-100数据（Heffley R. K., and Jewell N. F. Aircraft Handling Qualities Data – NASA CR 2144, Dec. 1972. 350 с.）。
