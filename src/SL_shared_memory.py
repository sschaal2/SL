""" 
Shared memory access for SL from Python

In order to access IPC shared memory objects, SL writes a special file into /tmp/
with all the shared memory objects. This file can be parsed into a dict, and then
used to access of the shared memory objects. Currently, this is used for connecting to
Python based simulators, like Isaac Gym.

Stefan Schaal, May 19, 2020             
"""

import ctypes, ctypes.util, sys, time, struct
import numpy as np


class SLSharedMemory:

    IPC_NOWAIT = 0o4000
    IPC_WAIT   = 0o0000
    EINTR      = 4
    EINTR      = 22
    EAGAIN     = 11
    ERROR      = -1
    
    def __init__(self,fname,robot):

        # error count
        self.errors = 0

        # the robot name
        self.robot = robot

        # read file and create dictionary of all objects
        self.shm_objects = {}

        with open(fname,"r") as f:
            for line in f:
                (name, type, id, key, size, offset) = line.split()
                self.shm_objects[name] = [int(type),int(id),int(key),int(size),int(offset),0,0]

        if len(self.shm_objects) < 1:
            sys.exit("cannot find shared memory objects in file",fname)

        # create all shared memory objects

        # try to find the operating system's libc
        libc_path = ctypes.util.find_library("c")
        if not libc_path:
            sys.exit("cannot find libc")

        # load libc
        libc = ctypes.CDLL(libc_path,use_errno=True)
        if not libc:
            sys.exit("cannot load libc")

        # map functions
        self.shmget = libc.shmget
        self.shmat  = libc.shmat
        # important to set the correct return type
        self.shmat.restype = ctypes.c_void_p
        self.semget = libc.semget
        self.semop  = libc.semop        

        # get the shared memory segments: this assumes these segments already exist and
        # were created by an SL master process
        for key in self.shm_objects:
            
            if self.shm_objects[key][0] == 3:  # this is shared memory
                shmid = self.shmget(self.shm_objects[key][2], self.shm_objects[key][3], 0)
                if shmid == self.ERROR:
                    sys.exit(print("cannot get shared memory segment for",key,self.shm_objects[key]))
                else:
                    self.shm_objects[key][5] = shmid

                ptr = self.shmat(shmid,0,0)
                if ptr == self.ERROR:
                    self.exit(print("problems with shmat: errno=%d\n",errno));
                else:
                    self.shm_objects[key][6] = ptr

                    
            elif self.shm_objects[key][0] == 0: # this is a binary shared semaphore
                semid = self.semget(self.shm_objects[key][2],1, 0)
                if semid == self.ERROR:
                    sys.exit(print("cannot get shared memory semaphore for",key,self.shm_objects[key]))
                else:
                    self.shm_objects[key][5] = semid


    # C compatible structure for semaphores: needed for semop calls
    class sembuffer(ctypes.Structure):
        _fields_ = [
            ('sem_num', ctypes.c_short),
            ('sem_op', ctypes.c_short),
            ('sem_flg', ctypes.c_short),            
        ]

    # vxWorks syntax for taking a semaphore
    def semTake(self,sem_name,timeout):

        name = self.robot+"."+sem_name
        
        sembuf         = self.sembuffer()
        sembuf.sem_num =  0;
        sembuf.sem_op  = -1;
        
        if timeout=='WAIT_FOREVER':
            
            sembuf.sem_flg =  self.IPC_WAIT
            
            while True:
                rc = self.semop(self.shm_objects[name][5],ctypes.pointer(sembuf),1)
                if rc != self.ERROR:
                    return True
                
                errno = ctypes.get_errno()
                if errno != self.EINTR:
                    sys.exit(print("SemTake(WAIT_FOREVER) exited with errno =",errno))
                    
        elif timeout=='NO_WAIT':
            
            sembuf.sem_flg =  self.IPC_NOWAIT
            rc = self.semop(self.shm_objects[name][5],ctypes.pointer(sembuf),1)
            if rc != self.ERROR:
                return True

            errno = ctypes.get_errno()
            if errno != self.EAGAIN:
                sys.exit(print("SemTake(NO_WAIT) exited with errno =",errno))

        else:

            count = int(float(timeout)/0.001)
            while --count > 0:
                sembuf.sem_flg =  self.IPC_NOWAIT
                rc = self.semop(self.shm_objects[name][5],ctypes.pointer(sembuf),1)        
                if rc != self.ERROR:
                    return True

                errno = ctypes.get_errno()
                if errno != self.EAGAIN:
                    sys.exit(print("SemTake(TIME_OUT) exited with errno =",errno))

                time.sleep(0.001)


            return False
                

    def semGive(self,sem_name):

        name = self.robot+"."+sem_name
        
        sembuf_array = self.sembuffer*2
        sembuf = sembuf_array();

        sembuf[0].sem_num =  0;
        sembuf[0].sem_op  =  0;
        sembuf[0].sem_flg =  self.IPC_NOWAIT;
        sembuf[1].sem_num =  0;
        sembuf[1].sem_op  =  1;
        sembuf[1].sem_flg =  0;

        rc = self.semop(self.shm_objects[name][5],ctypes.pointer(sembuf),2)

        if rc != self.ERROR:
            return True

        errno = ctypes.get_errno()
        if errno == self.EAGAIN:
            return True
        else:
            sys.exit(print("SemGive terminated with errno =",errno))


    # reads data from shared memory and returns all data in bytearray
    def smReadData(self,sm_name):

        name = self.robot+"."+sm_name        

        # data is stored at the pointer of shared memory + byte offset
        cptr      = ctypes.c_void_p(self.shm_objects[name][6]+self.shm_objects[name][4])
        length    = self.shm_objects[name][3]-self.shm_objects[name][4]
        data      = bytearray(length)
        data_ptr = (ctypes.c_char * length).from_buffer(data)        

        ctypes.memmove(data_ptr,cptr,length)

        return np.array(data, dtype=np.uint8)


    # converts a bytearray to a float array with prescribed number of columns and rows
    def bytesToFloatArray(self,data,nrows,ncols):

        # thhe follow creates a proper float array
        data_as_float = data.view(dtype=np.float32)
        data_array=np.reshape(data_as_float,(nrows,ncols))

        return data_array


    # write data to shared memory from a bytearray
    def smWriteData(self,sm_name,data_array):

        name = self.robot+"."+sm_name                

        # data is store at the pointer of shared memory + byte offset
        cptr      = ctypes.c_void_p(self.shm_objects[name][6]+self.shm_objects[name][4])
        length    = self.shm_objects[name][3]-self.shm_objects[name][4]
        data      = bytearray(data_array)
        data_ptr = (ctypes.c_char * length).from_buffer(data)        

        ctypes.memmove(cptr,data_ptr,length)


    #######################################################################
    # several specific shared memory read/write methods for specfic objects
    # note: the shared memrory layout is in SL_shared_memory.c/.h and not
    # understandable without looking into these files. There is also some
    # extra complexity in that SL uses vector/matrix indices starting with
    # 1 (from numerical receipe in C), and not with zero as in numpy, etc.
    #######################################################################    

    #######################################################################
    # broadcast the state of the simulator
    def send_sim_state(self,joint_state,base_state,base_orient,time_stamp):

        #### the joint state
        name      = 'smJointSimState'
        fname     =  self.robot+"."+name
        sem_name  = 'smJointSimState_sem'

        # put all data into the right size of an array as in shared memory
        ndofs,nelem        = np.shape(joint_state)
        data               = np.zeros((ndofs+2,nelem),dtype=np.float32)
        data[1:ndofs+1][:] = joint_state

        # need to prepend timestamp
        t=np.full((1),time_stamp,dtype=np.float32)
        float_array = np.concatenate((t,np.ravel(data)))

        # create byte array view
        byte_array  = float_array.view(dtype=np.int8)

        # should never happen
        if len(byte_array) != self.shm_objects[fname][3]-self.shm_objects[fname][4]:
            print("wrong data length",len(byte_array),self.shm_objects[fname][3]-self.shm_objects[fname][4])
        
        # take shared memory semaphore
        if self.semTake(sem_name,'1.0') == False:
            ++self.errors
            return False
            
        # copy to shared memory
        self.smWriteData(name,byte_array)
        
        # give shared memory semaphore
        if self.semGive(sem_name) == False:
            ++self.errors
            return False

        
        #### the base state
        name      = 'smBaseState'
        fname     =  self.robot+"."+name
        sem_name  = 'smBaseState_sem'

        # put all data into the right size of an array as in shared memory
        nrows,nelem            = np.shape(base_state)
        data                   = np.zeros((nrows*3,nelem),dtype=np.float32)
        data[nrows:nrows*2][:] = base_state

        # need to prepend timestamp -- the appending of t is a hack: it appears some 4 byte extra are needed for proper
        # memory allocation boundaries, like double byte alignment
        float_array = np.concatenate((t,np.ravel(data),t*0))

        # create byte array view
        byte_array  = float_array.view(dtype=np.int8)

        # should never happen
        if len(byte_array) != self.shm_objects[fname][3]-self.shm_objects[fname][4]:
            print("wrong data length",len(byte_array),self.shm_objects[fname][3]-self.shm_objects[fname][4])
        
        # take shared memory semaphore
        if self.semTake(sem_name,'1.0') == False:
            ++self.errors
            return False
            
        # copy to shared memory
        self.smWriteData(name,byte_array)
        
        # give shared memory semaphore
        if self.semGive(sem_name) == False:
            ++self.errors
            return False


        #### the base orient
        name      = 'smBaseOrient'
        fname     =  self.robot+"."+name
        sem_name  = 'smBaseOrient_sem'

        # put all data into the right size of an array as in shared memory
        nrows,nelem        = np.shape(base_orient)
        data               = np.zeros(((nrows*nelem-2)*3+1),dtype=np.float32)
        data[nrows*nelem-2+1:(nrows*nelem-2)*2+1] = \
                                      np.concatenate((base_orient[0],base_orient[1],base_orient[2],base_orient[3][0:4],base_orient[4][0:4]))
        data[0] = time_stamp

        # create byte array view
        byte_array  = data.view(dtype=np.int8)

        # should never happen
        if len(byte_array) != self.shm_objects[fname][3]-self.shm_objects[fname][4]:
            print("wrong data length",len(byte_array),self.shm_objects[name][3]-self.shm_objects[name][4])
        
        # take shared memory semaphore
        if self.semTake(sem_name,'1.0') == False:
            ++self.errors
            return False
            
        # copy to shared memory
        self.smWriteData(name,byte_array)
        
        # give shared memory semaphore
        if self.semGive(sem_name) == False:
            ++self.errors
            return False
        
        return True




    #######################################################################
    # receive the command for the simulator
    def receive_des_commands(self):

        name      = 'smDCommands'
        sem_name  = 'smDCommands_sem'

        # take shared memory semaphore
        if self.semTake(sem_name,'1.0') == False:
            ++self.errors
            return False
            
        # copy to shared memory
        byte_array =  self.smReadData(name)
        
        # give shared memory semaphore
        if self.semGive(sem_name) == False:
            ++self.errors
            return False

        return self.bytesToFloatArray(byte_array[4:],-1,5)
        
