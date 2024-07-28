import numpy as np

import crocoddyl
import eagle_mpc
import pinocchio as pin

class AerialSimulator():
    def __init__(self, robotModel, platformParams, dt, x0):
        self.robotModel = robotModel
        self.robotState = crocoddyl.StateMultibody(self.robotModel)  # 创建一个多体状态
        self.platformParams = platformParams
        self.dt = dt / 1000.

        self.actuationModel = crocoddyl.ActuationModelMultiCopterBase(self.robotState, self.platformParams.n_rotors,
                                                                      self.platformParams.tau_f)
        self.difAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.robotState, self.actuationModel, crocoddyl.CostModelSum(self.robotState, self.actuationModel.nu))
        self.intAM = crocoddyl.IntegratedActionModelRK4(self.difAM, self.dt)
        self.intAD = self.intAM.createData()
        
        #! ============================创建另外一个模型================================
        read_from_urdf = True  # 发现复制模型的方法不太好用，所以直接读取一个模型
        
        if read_from_urdf:
            print("Read from URDF")
            # 方法2：直接读取一个模型
            urdf_path = "/home/helei/catkin_ams/src/my_eagle_mpc_resources/example-robot-data/robots/hexacopter370_description/urdf/hexacopter370_flying_arm_3_with_load.urdf"
        
            self.robotModel2 = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
            self.robotState2 = crocoddyl.StateMultibody(self.robotModel2)
        else:
            # 方法1：直接复制一个模型
            self.robotModel2 = robotModel.copy()
            self.robotState2 = crocoddyl.StateMultibody(self.robotModel2)
            
            # 要修改的 link 名称
            target_link_name = "flying_arm_3__gripper"

            # 修改gripper的质量
            for frame in self.robotModel2.frames:
                if frame.type == pin.FrameType.BODY:
                    if frame.name == target_link_name:
                        # 修改 link 的属性，例如修改 inertia
                        frame.inertia.mass = 5.0  # 修改质量
                        # frame.inertia.inertia = pin.Inertia.Random().inertia  # 修改惯性矩
                        print(f"Modified Link: {frame.name}")
                        break
            # self.robotModel2.inertias[self.robotModel2.getJointId('flying_arm_3__gripper')].mass = 0.5  # TODO：但是flying_arm_3__gripper是一个link，不是一个joint，需要检查
        
        
        self.actuationModel2 = crocoddyl.ActuationModelMultiCopterBase(self.robotState2, self.platformParams.n_rotors,
                                                                      self.platformParams.tau_f)
        self.difAM2 = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.robotState2, self.actuationModel2, crocoddyl.CostModelSum(self.robotState2, self.actuationModel2.nu))
        self.intAM2 = crocoddyl.IntegratedActionModelRK4(self.difAM2, self.dt)
        self.intAD2 = self.intAM2.createData()

        self.x0 = x0
        self.states = [x0]
        self.controls = []

    def simulateStep(self, u):  
        self.controls.append(np.copy(u))
        self.intAM.calc(self.intAD, self.states[-1], self.controls[-1])  # 利用RK4对intAD进行更新
        self.states.append(np.copy(self.intAD.xnext))

        return self.states[-1]
    
    def simulateCatchStep(self, u, t, t_change):
        '''
        description: Change the model after t_change, t is the current time
        return {*}
        '''        
        print(f"Time: {t}")
        self.controls.append(np.copy(u))
        if t < t_change:
            self.intAM.calc(self.intAD, self.states[-1], self.controls[-1])
            self.states.append(np.copy(self.intAD.xnext))
        else: 
            print("Using another model")
            self.intAM2.calc(self.intAD2, self.states[-1], self.controls[-1]) # 从当前的状态，使用新的模型进行计算
            self.states.append(np.copy(self.intAD2.xnext))
            
        return self.states[-1]
