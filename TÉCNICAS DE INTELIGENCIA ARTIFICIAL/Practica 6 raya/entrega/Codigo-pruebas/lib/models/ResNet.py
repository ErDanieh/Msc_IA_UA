
import torch
import torch.nn as nn
import torch.nn.functional as F


class ResNet(nn.Module):
    
    def __init__(self, game, num_resBlocks, num_hidden, device=torch.device("cpu")):
        """Constructor de la estructura de la red neuronal

        Args:
            game (_type_): _description_
            num_resBlocks (int): Número de bloques residuales
            num_hidden (int): Número de capas ocultas
        """
        super().__init__()
        
        self.device = device
        
        # Bloque de entrada
        self.startBlock = nn.Sequential(
            nn.Conv2d(3, num_hidden, kernel_size=3, padding=1),
            nn.BatchNorm2d(num_hidden),
            nn.ReLU()
        )
        
        # Bloques residuales
        self.backBone = nn.ModuleList(
            [ResBlock(num_hidden) for _ in range(num_resBlocks)]
        )
        
        # Bloque de salida (policy head)
        self.policyHead = nn.Sequential(
            nn.Conv2d(num_hidden, 32, kernel_size=3, padding=1),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(32 * game.rows * game.cols, game.actionSize)
        )
        
        # Bloque de salida (value head)
        self.valueHead = nn.Sequential(
            nn.Conv2d(num_hidden, 3, kernel_size=3, padding=1),
            nn.BatchNorm2d(3),
            nn.ReLU(),
            nn.Flatten(),
            nn.Linear(3 * game.rows * game.cols, 1),
            nn.Tanh()
        )
        
        self.to(device)
        
    
    def forward(self, x):
        # Entrada
        x = self.startBlock(x)
        
        # Bloques residuales
        for resBlock in self.backBone:
            x = resBlock(x)
        
        # Salida
        policy = self.policyHead(x)
        value = self.valueHead(x)
        return policy, value

   
class ResBlock(nn.Module):
    
    def __init__(self, num_hidden):
        super().__init__()
        self.conv1 = nn.Conv2d(num_hidden, num_hidden, kernel_size=3, padding=1)
        self.bn1 = nn.BatchNorm2d(num_hidden)
        self.conv2 = nn.Conv2d(num_hidden, num_hidden, kernel_size=3, padding=1)
        self.bn2 = nn.BatchNorm2d(num_hidden)
    
    
    def forward(self, x):
        residual = x
        x = F.relu(self.bn1(self.conv1(x)))
        x = self.bn2(self.conv2(x))
        x += residual
        x = F.relu(x)
        return x