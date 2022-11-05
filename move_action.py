import numpy as np
import rotate_matrix as rot_m
import localenv.envloader as el
from time import sleep

class ur3eusing_example:
    def __init__(self):
        '''
        create environment
        '''
        self.base, env = el.loadEnv_wrs()
        rbt, rbtmg, rbtball = el.loadUr3e()
        rbtx = el.loadUr3ex(rbt)
        '''
        create robot
        '''

        self.robot_s = rbt
        self.robot_c =rbtx
        self.rbtmg=rbtmg
        self.arm_name = 'lft'


    def axis_line_move(self,distance,axis,planning=False , save=False, move=False,data_need=False):


        '''
        to place the robot in the ideal pos by human and read its positon
        '''
        if planning == True:

            start_jnts = self.robot_c.getjnts(armname=self.arm_name)
            start_jnts = np.array(start_jnts)
            self.robot_s.movearmfk(armname=self.arm_name, armjnts=start_jnts)
            self.rbtmg.genmnp(self.robot_s, toggleendcoord=True).reparentTo(self.base.render)
            [pos, rot_mat] = self.robot_s.getee(armname=self.arm_name)
            print('pos: ', pos)
            print("rot_mat: ", rot_mat)

            '''
            move to some place
            '''
            tgt_rot_mat = np.array([(0, 1, 0), (1, 0, 0), (0, 0, -1)])
            if axis=='x':
                tgt_pos=np.array([pos[0]+distance,pos[1],pos[2]])
            elif axis=='y':
                tgt_pos = np.array([pos[0], pos[1]+distance, pos[2]])
            elif axis=='z':
                tgt_pos = np.array([pos[0], pos[1], pos[2]+distance])
            else:
                tgt_pos = distance[0]
                tgt_rot_mat = distance[1]
            goal_jnts=self.robot_s.numikmsc(tgt_pos, tgt_rot_mat, seedjntagls = start_jnts, armname = self.arm_name)
            # self.robot_s.movearmfk(goal_jnts, armname=self.arm_name)
            # self.rbtmg.genmnp(self.robot_s, toggleendcoord=True).reparentTo(self.base.render)
            # self.base.run()
        if save == True:
            np.save("data/rotate_goal_jnts.npy", goal_jnts)

        if move == True:
            path=np.load("data/rotate_goal_jnts.npy")
            self.robot_c.movejntssgl(goal_jnts, armname=self.arm_name)
        if data_need == True:
            return goal_jnts, tgt_pos, tgt_rot_mat

    def rotate_move(self, angle, axis, planning=False , save=False, move=False, data_need=False):
        '''
         to place the robot in the ideal pos by human and read its positon
         '''

        if planning == True:
            start_jnts = self.robot_c.getjnts(armname=self.arm_name)
            start_jnts = np.array(start_jnts)
            self.robot_s.movearmfk(armname=self.arm_name, armjnts=start_jnts)
            self.rbtmg.genmnp(self.robot_s, toggleendcoord=True).reparentTo(self.base.render)
            [pos, rot_mat] = self.robot_s.getee(armname=self.arm_name)
            print('pos: ', pos)
            print("rot_mat: ", rot_mat)

            '''
            move to some place
            '''
            tgt_pos = pos
            rotate_matrix=rot_m.matrix_generate(angle, axis)
            tgt_rot_mat = np.dot(rot_mat, rotate_matrix)

            goal_jnts = self.robot_s.numikmsc(tgt_pos, tgt_rot_mat, seedjntagls = start_jnts, armname = self.arm_name)
            self.robot_s.movearmfk(goal_jnts, armname=self.arm_name)
            self.rbtmg.genmnp(self.robot_s, toggleendcoord=True).reparentTo(self.base.render)
            # self.base.run()
        if save == True:
            np.save("data/rotate_goal_jnts.npy", goal_jnts)

        if move == True:
            path=np.load("data/rotate_goal_jnts.npy")
            self.robot_c.movejntssgl(goal_jnts, armname=self.arm_name)

        if data_need == True:
            return goal_jnts, tgt_pos, tgt_rot_mat

    def start_pos(self):
        '''
         to place the robot in the ideal pos by human and read its positon
         '''

        start_jnts = self.robot_c.getjnts(armname=self.arm_name)
        start_jnts = np.array(start_jnts)
        self.robot_s.movearmfk(armname=self.arm_name, armjnts=start_jnts)
        self.rbtmg.genmnp(self.robot_s, toggleendcoord=True).reparentTo(self.base.render)
        [pos, rot_mat] = self.robot_s.getee(armname=self.arm_name)
        print('pos: ', pos)
        print("rot_mat: ", rot_mat)

        '''
        move to some place
        '''
        tgt_pos = pos
        tgt_rot_mat = np.array([(0, 1, 0), (1, 0, 0), (0, 0, -1)])

        # tgt_rot_mat=rot_mat
        goal_jnts = self.robot_s.numikmsc(tgt_pos, tgt_rot_mat, seedjntagls = start_jnts, armname = self.arm_name)

        np.save("data/start_jnts.npy", goal_jnts)
        print("Successfully save the path!")

        self.robot_c.movejntssgl(goal_jnts, armname=self.arm_name)

        return goal_jnts

    def just_move(self,goal_jnts, data_need=False):
        self.robot_c.movejntssgl(goal_jnts, armname=self.arm_name)
        if data_need:
            self.robot_s.movearmfk(armname=self.arm_name, armjnts=goal_jnts)
            self.rbtmg.genmnp(self.robot_s, toggleendcoord=True).reparentTo(self.base.render)
            [pos, rot_mat] = self.robot_s.getee(armname=self.arm_name)
            return pos, rot_mat

if __name__ == '__main__':
    ma_ex=ur3eusing_example()
