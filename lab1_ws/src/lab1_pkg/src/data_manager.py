import os
import numpy as np
from numpy.random import random
import scipy.io as sio
# import cv2

# import copy
# import glob

# import pickle
# import IPython



class data_manager(object):
    def __init__(self,folder_name):

        #Batch Size for training
        self.batch_size = 40
        #Batch size for test, more samples to increase accuracy
        self.val_batch_size = 100

        # To keep track of where in the data you are in the training data
        self.cursor = 0
        # Same as above but for validation data
        self.t_cursor = 0 

        self.epoch = 1
        # split the collected data into training data and validation data
        # file name containing features should be 'inputValue'
        # file name containing labels should be 'angle_pos'
        # 'key' of the dictionary should be the same as the file name

        # self.split_data_into_train_cal(self.load_set(folder_name))
        self.train_data = self.load_set(folder_name)


    def get_train_batch(self):

        '''

        Compute a training batch for the neural network 
        The batch size should be size 40

        '''
        if (self.cursor + self.batch_size > len(self.train_data)):
            np.random.shuffle(self.train_data)
            self.cursor = 0

        train_batch_list = self.train_data[self.cursor : self.cursor+self.batch_size]
        train_batch_label = [dic['label'] for dic in train_batch_list]
        train_batch_features = [dic['features'] for dic in train_batch_list]
        self.cursor = self.cursor + self.batch_size
        
        return np.array(train_batch_features) , np.array(train_batch_label)


    def get_validation_batch(self):

        '''
        Compute a training batch for the neural network 

        The batch size should be size 400

        '''
        #FILL IN
        if (self.t_cursor + self.val_batch_size > len(self.val_data)):
            np.random.shuffle(self.val_data)
            self.t_cursor = 0

        val_batch_list = self.val_data[self.t_cursor : self.t_cursor+self.val_batch_size]
        val_batch_label = [dic['label'] for dic in val_batch_list]
        val_batch_features = [dic['features'] for dic in val_batch_list]
        self.t_cursor = self.t_cursor + self.val_batch_size

        return np.array(val_batch_features) , np.array(val_batch_label)


    def load_set(self,folder_name):

        '''
        Given a string which is either 'val' or 'train', the function should load all the
        data into an 

        '''
        # folder = 'data/'
        filename = 'inputValue'
        x = sio.loadmat(folder_name+filename+'.mat')[filename]
        filename = 'threeD_pos'
        y = sio.loadmat(folder_name+filename+'.mat')[filename]
        # np.delete(y,1,1)
        data = [ {'features':x[i,:] , 'label':y[i,0:3]} for i in range(x.shape[0]) ]

        np.random.shuffle(data)

        return data


    def split_data_into_train_cal(self,data):
        # need 'data' to be a list
        split_pos = int(len(data)*5.0/6.0)
        self.train_data = data[0:split_pos]
        self.val_data = data[split_pos:]
        self.batch_size = len(self.train_data)
        self.val_batch_size = len(self.val_data)

    def rescale(self,data):
        # rescale data to [-pi,pi]
        for i in range(4):
            data[:,i*3+2] = (data[:,i*3+2]-52.5)/37.5*3.14
            data[:,i*3] = (data[:,i*3]-320.0)/320.0*3.14
            data[:,i*3+1] = (data[:,i*3+1]-240.0)/240.0*3.14

        return data
