import torch
import torch.nn as nn
import torch.nn.functional as F

import torchvision.models as models

class Decoder(nn.Module):
    def __init__(self):
        super(Decoder, self).__init__()
        
        self.alpha = nn.Sequential(
            nn.Linear(512,1),
            nn.Sigmoid()
        )
        
        self.fc = nn.Linear(512,1)
    

    def forward(self, x):
                
        a = self.alpha(x)
        x = x.mul(a).sum(1).div(a.sum(1))
        x = self.fc(x)
        x = torch.tanh(x)
        
        return x


class Encoder(nn.Module):
    def __init__(self):
        super(Encoder, self).__init__()
        
        resnet = models.resnet18(pretrained=True) 
        self.resnet = nn.Sequential(*list(resnet.children())[:-1])

    def forward(self, X):
        
        t = X.size(1)
        
        r = []
        
        for i in range(t):
            
            x = X[:,i,:,:,:]
            x = self.resnet(x)
            
            r.append(x)
        
        X = torch.stack(r,dim=1)
        
        return X.squeeze(4).squeeze(3)
    
class Model(nn.Module):
    def __init__(self):
        super(Model, self).__init__()
        
        self.encoder = Encoder()
        self.decoder = Decoder()
    
    def forward(self, X):
        X = self.encoder(X)
        X = self.decoder(X)
        return X