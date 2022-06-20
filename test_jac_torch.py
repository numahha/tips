import torch

model = torch.nn.Sequential(
            torch.nn.Linear(2, 2)
        )

x = torch.tensor([[0.,0.]], dtype=torch.float32)
x.requires_grad = True
preds = model(x)
J = torch.zeros ((1, 2, 2))
for i in range(2):
    grd = torch.zeros ((1,2))
    grd[0, i] = 1
    preds.backward (gradient = grd, retain_graph = True)
    J[:,:,i] = x.grad
    x.grad.zero_()

print("model",model.state_dict()['0.weight'])
print("J",J[0].T)
