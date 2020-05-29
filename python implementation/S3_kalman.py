# -*- coding: utf-8 -*-
"""
Created on Wed May 20 21:57:51 2020

@author: Armando
"""


import numpy as np
import pandas as pd

import matplotlib.pyplot as plt

from filterpy.stats import plot_covariance_ellipse
from time import sleep
from mpl_toolkits import mplot3d
from scipy.stats import uniform

class Kalman:
    
    def __init__(self,X,Q,R,P):
        self.X=X
        self.Q=Q
        self.R=R
        self.P=P
        
        
    
    def update(self,Z,H):
        
        Y = Z - np.matmul(H.T,self.X)
        S = np.matmul(H.T,np.matmul(self.P,H))+self.R
        temp1=np.matmul(H,1/S)
        #np.matmul(H.T,np.linalg.inv(S))
        K = np.matmul(self.P,temp1)

        self.X= self.X+ np.matmul(K,Y)
        #self.X= self.X+ np.matmul(K.T,Y)
        temp= np.matmul(K,H.T)
        self.P=np.matmul((np.eye(temp.shape[0])-temp),self.P)
        return (self.X,self.P,K,Y)
        
        
        
    def predict(self,F,B,U):
        
        self.X = np.matmul(F,self.X)+np.matmul(B,U)
        self.P = np.matmul(F,np.matmul(self.P,F.T))+self.Q
        return (self.X,self.P)
        
    
    
    
    
    
    
if __name__ == "__main__":
    """ Executed only when the file is run as a script. """
    

    np.random.seed(123)
    
    
    
    for q in [3.0,10]:
        print(q)
        
        for r in [3,10]:
            print(r)
            
            for x in [[[0],[0]],[[2],[6]]]:
                print(x)
                # Initialization of state matrices
                
                #X = np.array([[0],[0]])             # initial state E(X(0))=0
                X=x
                P = np.diag((40,40))                # initial state covariance
                F = np.array([[0.8,2], [0,0.9]])    # the state-transition model
                Q = np.eye(2)*q                # the covariance of the process noise
                B = np.diag((1,1))                 # the control-input model
                U = np.array([[3],[5]])            # control vector
                R=r
                
            
                Ek=np.array([0,0])                  #Controll input model
                  
                
                G=np.diag((1,1))
                #G=np.array([[1],[1]])
                
                # Measurement matrices    
                H = np.array([[1], [1]])                #the observation model
                #R = np.diag((6,6))                     #the covariance of the observation noise
                
                
                # number of iteration
                N_iter=50
                
                # outputs of Kalman update
                outputX=[]
                outputP=[]
                outputK=[]
                outputY_hat=[]
                
                #Kalman Predict
                
                X_pre=[]
                P_pre=[]
                
                
    
                
                #init Kalman Filter
                kalman_filter = Kalman(X,Q,R,P)
            
                outputX.append(X)
                outputP.append(P)
                
                # lists for the motion model Xk,Y
                ListXk=[]
                ListY=[]
                
                Xk=X
                
                ListXk.append(X)
                
            
            
            
                # applying kalman filter
                for t in range (0,N_iter):
                    
                    w1=np.random.uniform(-3, 3, 1)
                    w2=np.random.uniform(-3, 3, 1)
                    
                    # process noise
                    W =np.array([[w1[0]],[w2[0]]])
                
                    e=np.random.uniform(-3, 3, 1)
                    
                    # motion model
                    Xk=np.matmul(F,Xk)+np.matmul(B,U)+np.matmul(G,W)
                           
                    ListXk.append(Xk)
                    
                    # measurement model
                    Y=np.matmul(H.T,Xk)+np.matmul(Ek,U)+e
                    #Y=np.matmul(np.diag((1,1)),Xk)+np.matmul(Ek,U)+e
                    
                    ListY.append(Y)
                    
                    
                    # Kalman predict
                    (X,P)=kalman_filter.predict(F,B,U)
                    
                    #plot_covariance_ellipse(X, P, edgecolor='b')
                    
                    X_pre.append(X)
                    P_pre.append(P)
                    
                    
                            
                    #Kalman update
                    (X,P,K,Y_hat)=kalman_filter.update(Y,H)
                    
                    
                    #plot_covariance_ellipse(X, P, edgecolor='r')
                   
                    
                    #ax1.plot(Y,'-bx')
                    
                    
                    outputX.append(kalman_filter.X)
                    outputP.append(kalman_filter.P)
                    outputK.append(K)
                    outputY_hat.append(Y_hat)
                    
                    '''
                    ax1.set_title('Kalman filter iter=%d'%(t+1))
                    plt.xlabel('X1')
                    plt.ylabel('X2')
                    plt.legend(['Kalman Predict','Kalman Update'])
                    ax1.relim()
                    ax1.autoscale_view()
            
                    sleep(0.05)
                    fig.canvas.draw()
                    fig.canvas.flush_events()
                    #print(t)
                    '''
                    
                #Xk
                list_x1=[]
                list_x2=[]
                #outputX
                list_x1_hat=[]
                list_x2_hat=[]
                #Kalman Gain
                gain=[]
                
                
                
                for i in range(N_iter):
                    
                    list_x1.append(ListXk[i][0][0])
                    list_x2.append(ListXk[i][1][0])
                    
             
                    list_x1_hat.append(outputX[i][0][0])
                    list_x2_hat.append(outputX[i][1][0])
                    
                    gain.append(outputK[i][0])
                    
                    
                
                N=[i for i in range(0,50)]
                
                
                
                
                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                
                #plt.plot(list_x1,'-r.',list_x2,'-b.')
                plt.plot(list_x1,'-r.')
                plt.plot(list_x2,'-b.')
                plt.figtext(0.6,0.9,'Q ='+str(q)+', R = '+str(r)+', X ='+str(x))
                plt.legend(['X1','X2'])
                
                
                
                
                '''
                fig= plt.figure()
                fig = plt.figure()
                ax1 = fig.add_subplot(111)
                
                #plt.plot(list_x1,'-r.',list_x2,'-b.')
                plt.plot(list_x1,'-r.')
                '''
                
                
                
                #plt.legend([,'Kalman Update'])
                #plt.plot(N,listy_hat,N,list_y)
                #plt.plot(list_x1,'b',list_x1_hat,'r')
                
                
                '''
                plt.figure(figsize=(7, 5))
                plt.plot(list_x1,list_x2,'-bo')
                plt.plot(list_x1_hat, list_x2_hat, '-rx')
                #plt.plot(list_y1, list_y2, '-y.')
                plt.xlabel('x [m]')
                plt.ylabel('y [m]')
                plt.legend(['true state','inferred state'])
                plt.axis('square')
                plt.tight_layout(pad=0)
                plt.show()
                '''
        
        '''
        #plt.plot(N,list_x1,N,list_x1_hat)
        #plt.plot(N,list_x2,N,list_x2_hat,N,list_y)    
         
        #plt.plot(N,gain)
        ListXk=[i for i in range(0,50)]
        outputX=[i for i in range(0,50)]
        
        #plt.plot(N,ListXk,N,outputX)
    
        
        
    
        '''
    
    
        
    