# Network Model for learning difficulty assesment of mazes given a players performance
# Network will serve as a proof of concept for My masters project
# Inputs: Time, Maze Size, Num of Collisions, Num of coins, Num of Deaths
# Outpus
# Author: Robbie Riviere

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
import torchvision
import torchvision.transforms as transforms
from torchvision.transforms import ToTensor
from torchvision import datasets
from torch.utils.data import DataLoader

print("running")
z = torch.zeros(5, 3)
print(z)
print(z.dtype)