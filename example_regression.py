import numpy as np 
import matplotlib.pyplot as plt
import torch
from torch.utils.data import TensorDataset, DataLoader


x_train = np.linspace(-2,2, 200, dtype=np.float32)
y_train = x_train**2 + np.random.randn(x_train.shape[0])*x_train

x_train = torch.from_numpy(x_train)
y_train = torch.from_numpy(y_train)
x_train = x_train.reshape(x_train.shape[0],1)
y_train = y_train.reshape(y_train.shape[0],1)


class Model(torch.nn.Module):
    def __init__(self, D_in=1, D_out=1, hidden_unit_num=16):

        super(Model, self).__init__()
        self.D_out = D_out
        self.l1 = torch.nn.Linear(D_in, hidden_unit_num)
        self.l2 = torch.nn.Linear(hidden_unit_num, 2*D_out)

    def forward(self, X):
        mu_logvar = self.l2(torch.tanh(self.l1(X)))
        return mu_logvar[:,:self.D_out], mu_logvar[:,self.D_out:]


loader = DataLoader(TensorDataset(x_train, y_train), batch_size=32, shuffle=True,  drop_last=True)


model = Model()
optimizer = torch.optim.Adam(model.parameters(), lr=5.e-3)

for i in range(1000):
    train_loss =0.
    for x_batch, y_batch in loader:
        optimizer.zero_grad()
        mu, logvar = model(x_batch)
        inv_var = torch.exp(-logvar)
        loss = ( inv_var * ((mu-y_batch)**2) + logvar  ).sum()
        loss.backward()
        train_loss += loss.item()
        optimizer.step()
    print(i, train_loss)


plt.plot(x_train,y_train,"o")
with torch.no_grad():
    y_pred, y_logvar = model(x_train)

y_sample = y_pred + torch.exp(0.5*y_logvar) * torch.randn(y_pred.shape)
y_sample2 = y_pred + torch.exp(0.5*y_logvar) 
y_sample3 = y_pred - torch.exp(0.5*y_logvar)
plt.plot(x_train,y_pred,"s")
plt.plot(x_train,y_sample2,"x")
plt.plot(x_train,y_sample3,"x")
plt.show()


