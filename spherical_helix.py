import numpy as np

import motionplanner.motion_planner as m_planner
import motionplanner.rbtx_motion_planner as m_plannerx
import utils.phoxi as phoxi
import utils.phoxi_locator as pl
import utiltools.robotmath as rm
from utils.run_script_utils import *
import environment.collisionmodel as cm
import localenv.envloader as el


# when the error is 9mm, length is 53mm, the joint angle is [-178.4940885432176, -15.216969128955634, 63.26866064131909, 146.63225895317703, -2.313074525647169, -120.72505888292342]

if __name__ == '__main__':
    '''
    set up env and param
    '''
    base, env = el.loadEnv_wrs()
    rbt, rbtmg, rbtball = el.loadUr3e()
    rbtx = el.loadUr3ex(rbt)
    # rbt.opengripper(armname="rgt")
    rbt.opengripper(armname="rgt")
    # table = cm.CollisionModel("../obstacles/ur3edtable.stl")
    # table.reparentTo(base.render)
    pen = el.loadObj(config.PEN_STL_F_NAME)
    # cube = el.loadObjitem('cube.stl', pos=(600, 200, 780))
    # cube.show_objcm()

    '''
    init class
    '''
    # # mp_rgt = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="rgt")
    mp_lft = m_planner.MotionPlanner(env, rbt, rbtmg, rbtball, armname="rgt")
    # # mp_x_rgt = m_plannerx.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="rgt")
    # # mp_x_lft = m_plannerx.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, armname="lft")
    mp_x_lft = m_plannerx.MotionPlannerRbtX(env, rbt, rbtmg, rbtball, rbtx, armname="rgt")

    base.pggen.plotAxis(base.render, spos=[0, 0, 0], length=5000)

    def moveupward(realrbt = False, armname = "lft"):
        # target_jnts1 = [-325.57158112999906, -149.47337041810468, -99.73188826034412, 249.2868329706938, -145.579803083898, -269.93851896280256]
        target_jnts1 = [125.96878271157331, -52.529848602439245, -1.0491675358807933, -138.26519391465385, 100.90092294811599,
         26.907079442780013]
        rbt.movearmfk(target_jnts1, armname=armname)
        rbtmg.genmnp(rbt, toggleendcoord=True).reparentTo(base.render)
        if realrbt:
            rbtx.movejntssgl(target_jnts1, armname=armname)
        print("finish move upward")
        # base.pggen.plotAxis(base.render, spos=[0, 0, 0], length=5000)

    def movedownward(realrbt = False, armname = "rgt"):
        target_jnts2 = [-319.2733270925469, -150.38719508753903, -96.43691621227076, 246.89222328920067, -139.2917396886875, -269.9390790383145]
        rbt.movearmfk(target_jnts2, armname=armname)
        rbtmg.genmnp(rbt, toggleendcoord=True).reparentTo(base.render)
        if realrbt:
            rbtx.movejntssgl(target_jnts2, armname=armname)
        print("finish move downward")

    def aligngravity(realrbt = False, armname = "rgt"):
        # start_jnts = [-178.7583895433404, 44.09971309537172, -108.40588016661263, 249.79937036601154, -1.2066112008617835, 264.5153041382117]
        start_jnts = rbtx.getjnts(armname=armname)
        # print("start_jnts",start_jnts)
        rbt.movearmfk(start_jnts, armname=armname)
        rbtmg.genmnp(rbt, toggleendcoord=True).reparentTo(base.render)

        # pos = [623.18070112, 72.39287886, 912.05719441]
        # rot = [[ 9.99997960e-01, -1.27353274e-04, -2.01581244e-03],[-1.25927238e-04, -9.99999742e-01, 7.07536082e-04],[-2.01590203e-03, -7.07280793e-04, -9.99997718e-01]]
        pos, rot = rbt.getee(armname=armname)
        print("pos",pos)
        print("rot",rot)
        # pos_tcp = [ 623.56168967,   72.25915454, 1101.0567631 ]
        # rot_tcp = [[9.99997960e-01, - 1.27353274e-04, - 2.01581244e-03],[-1.25927238e-04, - 9.99999742e-01, 7.07536082e-04],[-2.01590203e-03, - 7.07280793e-04, - 9.99997718e-01]]
        pos_tcp, rot_tcp = rbt.gettcp(armname=armname)
        print("pos_tcp",pos_tcp)
        print("rot_tcp",rot_tcp)
        diff = pos-pos_tcp
        print("diff", diff)
        rot_aligned = rm.rodrigues((0,0,1),180).dot(rm.rodrigues((0,1,0),180))
        # rot_aligned = rm.rodrigues((0,0,1),180).dot(rm.rodrigues((0, 1, 0), 180))
        goal_pos = pos
        goal_rot = rot_aligned
        goal_jnts1 = rbt.numikmsc(goal_pos, goal_rot, seedjntagls = start_jnts, armname = armname)

        rbt.movearmfk(goal_jnts1, armname=armname)
        rbtmg.genmnp(rbt, toggleendcoord=True).reparentTo(base.render)
        if realrbt:
            rbtx.movejntssgl(goal_jnts1, armname=armname)
        print("finish gravity alignment")
        return (goal_jnts1)

    def incline_rot(realrbt = False, armname = "rgt"):
        start_jnts2 = rbtx.getjnts(armname=armname)
        # start_jnts2 = [-178.7598512038228, 44.09695369894703, -108.40877616682073, 249.79863270558118, -1.2065838801051034, 264.5166974968024]
        rbt.movearmfk(start_jnts2, armname=armname)
        rbtmg.genmnp(rbt, toggleendcoord=True).reparentTo(base.render)
        pos2, rot2 = rbt.getee(armname=armname)
        # def getinclination(circle, angle_incline = 10, axis_incline = (0,1,0)):
        # world_homo_list.append(getinclination(temp_axis, angle_incline, (0, 1, 0)))
        # circle is a rotation matrix that rotates around z axis
        def getinclination(circle, angle_incline, axis_incline=(0, 1, 0)):

            start_homo = rm.homobuild(pos2, rot2)
            # 两个矩阵相乘，incline_axis是指经过旋转矩阵变换后的y轴
            incline_axis = np.dot(circle, axis_incline)
            # 绕着新的y轴进行旋转操作
            incline_rotmat = rm.rodrigues(incline_axis, angle_incline)

            # objoffset = 320 - 189
            objoffset = 320 - 189
            coordi = rm.homobuild(pos2 + np.array([0, 0, -objoffset]), np.eye(3))
            # 矩阵求逆
            relativecor_homo_inv = np.linalg.inv(coordi)
            # 加入柔性手腕之后ee的坐标
            relative_homo = relativecor_homo_inv.dot(start_homo)
            # 绕y轴进行旋转操作
            afterrot_homo = rm.homobuild(pos=[0, 0, 0], rot=incline_rotmat).dot(relative_homo)
            # 进行旋转后再延z轴负方向移动到虚拟ee的位置
            world_homo = coordi.dot(afterrot_homo)
            # print(world_homo[3,:3])
            base.pggen.plotAxis(base.render, spos=world_homo[:3,3], srot=world_homo[:3,:3], length=500)
            base.pggen.plotAxis(base.render, spos=[0,0,0], srot=world_homo[:3, :3], length=5000)
            return world_homo

        world_homo_list = []
        path_list = [start_jnts2]

        # 需要将旋转轴平移到peg的末端
        # temp_axis = rm.rodrigues((0, 0, 1), 30)
        # world_homo_list.append(getinclination(temp_axis))
        # for i in range(0,72,1):
        #     temp_axis = rm.rodrigues((0,0,1),i*10)
        #     angle_incline = i/5
        #     world_homo_list.append(getinclination(temp_axis, angle_incline, (0,1,0)))
        n = 36
        for i in range(n):
            angle_incline = i*3/n
            i = i%36
            temp_axis = rm.rodrigues((0,0,1),i*10)
            # for i in range(144):
            #     angle_incline = i / 20
            #     i = i % 36
            #     temp_axis = rm.rodrigues((0, 0, 1), i * 10)

            world_homo_list.append(getinclination(temp_axis, angle_incline, (0,1,0)))

        # for i in range(37,72,1):
        #     temp_axis = rm.rodrigues((0,0,1),i*5)
        #     angle_incline = i/5
        #     world_homo_list.append(getinclination(temp_axis, angle_incline, (0,1,0)))

        # base.run()
        # print(world_homo_list)
        for j, homo in enumerate(world_homo_list):
            goal_pos = homo[:3,3]
            goal_rot = homo[:3,:3]
            # print("goal_pos",goal_pos)
            base.pggen.plotAxis(base.render, spos=goal_pos, srot=goal_rot, length=200)

            goal_jnts = rbt.numik(goal_pos, goal_rot, seedjntagls=path_list[j], armname=armname)
            print(j)
            # goal_jnts = rbt.numikmsc(goal_pos, goal_rot, seedjntagls=start_jnts2, armname=armname)
            try:
                rbt.movearmfk(goal_jnts,armname=armname)
                rbtmg.genmnp(rbt, toggleendcoord=True).reparentTo(base.render)
                path_list.append(goal_jnts)
            except:
                break

        if realrbt:
            rbtx.movejntssgl(path_list[0],armname = armname)
            mp_x_lft.movepath_with_sm(path_list[0:])
            # rbtx.movejntssgl(goal_jnts, armname=armname)
        # 1
    moveupward(True)
    # movedownward(True)
    # aligngravity(True)
    # incline_rot(True)

    # moveupward(True)
    # movedownward(True)
    # aligngravity(True)
    # incline_rot(True)
    #
    # moveupward(True)
    # movedownward(True)
    # aligngravity(True)
    # incline_rot(True)
    #
    # moveupward(True)
    # movedownward(True)
    # aligngravity(True)
    # incline_rot(True)
    #
    # moveupward(True)
    # movedownward(True)
    # aligngravity(True)
    # incline_rot(True)
    base.run()
    # mp_x_lft.goto_init_x()
    # mp_x_rgt.goto_init_x()
    # init_armjnts = mp_x_lft.get_armjnts()
    # path = []
    # planning
    # 1
    # for i in range(0, 360, 60):
    #     path_out = mp_lft.get_moveup_path(init_armjnts, pen, np.asarray([0, 0, 0]), np.eye(3),
    #                                       direction=[np.cos(i), np.sin(i), 0], length=20)
    #     path.extend(path_out)
    #     path_up = mp_lft.get_moveup_path(path_out[-1], pen, np.asarray([0, 0, 0]), np.eye(3),
    #                                      direction=[0, 0, 1], length=20)
    #     path.extend(path_up)
    #     # path_in = mp_lft.get_moveup_path(path_up[-1], pen, np.asarray([0, 0, 0]), np.eye(3),
    #     #                                  direction=[-np.cos(i), -np.sin(i), 0], length=20)
    #     # path.extend(path_in)
    #     # path_down = mp_lft.get_moveup_path(path_in[-1], pen, np.asarray([0, 0, 0]), np.eye(3),
    #     #                                  direction=[0, 0, -1], length=20)
    #     # path.extend(path_down)
    # 1
    # mp_lft.ah.show_animation(path)
    # base.run()

    # transfer to robot
    # mp_x_lft.movepath(path)

    # force control
    # mp_x_lft.force_controller.forcemode()
    # mp_x_lft.force_controller.passive_move(path, np.eye(4))
    # base+animtion    show animation