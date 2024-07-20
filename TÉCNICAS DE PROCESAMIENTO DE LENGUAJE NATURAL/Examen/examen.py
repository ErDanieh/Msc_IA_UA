import torch

x = torch.ones(10,5)
print(x)
print(x.shape)

print("-----------------------------------------------")
y = torch.randn(5, 20)
print(y)
print(y.shape)

print("-----------------------------------------------")
z = torch.matmul(x, y)
print(z)
print(z.shape)

print("-----------------------------------------------")
z = z.view(2, -1)
print(z)
print(z.shape)

print("-----------------------------------------------")
z = torch.nn.functional.relu(z)
print(z)
print(z.shape)

print("-----------------------------------------------")
batch_size = 2
z = z.view(batch_size, -1, 10)
print(z)
print(z.shape)

print("-----------------------------------------------")
z = z.mean(dim=0)
print(z)
print(z.shape)