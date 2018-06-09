# -*- coding: utf-8 -*-
"""
Created on Sat Jun  9 11:04:28 2018

@author: mahbub
"""

import plotly.plotly as py
from plotly.tools import FigureFactory as FF
from plotly.graph_objs import *

from datetime import datetime
from pandas_datareader import data, wb
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

import seaborn as sns
from sklearn.datasets import make_classification
from sklearn.ensemble import ExtraTreesClassifier,RandomForestClassifier,AdaBoostClassifier,RandomForestRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from sklearn.model_selection import cross_val_score
from sklearn.datasets import make_blobs
from sklearn.tree import DecisionTreeClassifier,DecisionTreeRegressor
from sklearn.neighbors import KNeighborsClassifier
from sklearn.metrics import confusion_matrix, precision_score, mean_squared_error, recall_score, f1_score, cohen_kappa_score
from plotly.offline import download_plotlyjs, init_notebook_mode, plot, iplot
import csv


class Stock(object):
    symbol=''
    MaxTimeGap=0
    Volume=0
    WeightedAveragePrice=0
    MaxPrice=0
    totalPrice = 0
    timestamp=0

    def __init__(self, timestamp,name,volume,price):
        self.symbol = name
        self.MaxTimeGap = 0
        self.Volume = (int)(volume)
        self.WeightedAveragePrice = (int)(price)
        self.MaxPrice = (int)(price)
        self.totalPrice = (int)(price)*(int)(volume)
        self.timestamp=(int)(timestamp)
  

    def update(self, timestamp,volume,price):
        """updates the attributes with new trades"""
        self.MaxTimeGap = max(self.MaxTimeGap,((int)(timestamp)-self.timestamp))
        self.timestamp=(int)(timestamp)
        self.Volume = self.Volume+(int)(volume)
        self.totalPrice = self.totalPrice+(int)(price)*(int)(volume)
        self.WeightedAveragePrice =(int)(self.totalPrice/self.Volume) 
        self.MaxPrice = max((int)(price),self.MaxPrice)
        return

    


data={}

chunksize = 1000

"""read 1000 chank at a time    """    
for df in pd.read_csv("input.csv",header=None,chunksize=chunksize, iterator=True):
    aList=df.values.tolist()
    for r in aList:
        if r[1] in data:
            aStock=data[r[1]]
            aStock.update(r[0],r[2],r[3])
        else:
            data[r[1]]=Stock(r[0],r[1],r[2],r[3])
 


"""output the calculated values    """       
res=[]          
for name in data:
    aStock=data[name]
    res.append([aStock.symbol,aStock.MaxTimeGap,aStock.Volume,aStock.WeightedAveragePrice,aStock.MaxPrice])           
            
with open("output.csv", "w") as output:
    writer = csv.writer(output, lineterminator='\n')
    writer.writerows(res)            
            
            
            
            
    