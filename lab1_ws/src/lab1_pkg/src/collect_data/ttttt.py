
from backprop import *
from sklearn.linear_model import Ridge


def load_model(num_layers,path1,path2,path3,path4):
    model_W = []
    model_b = []
    model_ridge_WW = np.loadtxt(path3)
    model_ridge_W = np.reshape(model_ridge_WW,(model_ridge_WW.shape[0],-1))
    model_ridge_ii = np.loadtxt(path4)
    model_ridge_intercept = np.reshape(model_ridge_ii,(model_ridge_ii.shape[0],-1))
    for i in range(num_layers):
        WW = np.loadtxt(path1.format(i))
        model_W.append( np.reshape(WW,(WW.shape[0],-1)) )
        bb = np.loadtxt(path2.format(i))
        model_b.append( np.reshape(bb,(bb.shape[0],-1)) )
    return model_W , model_b, model_ridge_W, model_ridge_intercept

folder = './data/'
filename = 'inputValue'
x = sio.loadmat(folder+filename+'.mat')[filename]
filename = 'angle_pos'
y = sio.loadmat(folder+filename+'.mat')[filename]

# create models
activations = dict(ReLU=ReLUActivation,
                   tanh=TanhActivation,
                   linear=LinearActivation)
activation = activations['ReLU']
model = Model(12)
for i in range(3):
    model.addLayer(DenseLayer(95,activation()))
model.addLayer(DenseLayer(7,LinearActivation()))
model.initialize(QuadraticCost())
ridge = Ridge(alpha=0.1)
# load model parameters
path1 = './model/my_model_W_{}.dat'
path2 = './model/my_model_b_{}.dat'
path3 = './model/my_model_ridge_W.dat'
path4 = './model/my_model_ridge_intercept.dat'
model_W , model_b, model_ridge_W, model_ridge_intercept = load_model(4,path1,path2,path3,path4)
# update the model
for i in range(len(model.layers)):
	model.layers[i].W = model_W[i]
	model.layers[i].b = model_b[i]
ridge.coef_ = model_ridge_W
ridge.intercept_ = model_ridge_intercept[0]
print(ridge.intercept_.shape)

# predict the configuration
y1 = model.predict(x)
tar_position_in_list = ridge.predict(y1)
print(tar_position_in_list[0:10])
