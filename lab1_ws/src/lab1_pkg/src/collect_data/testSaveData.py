import scipy.io as sio
inputValue = sio.loadmat('./new_data/inputValue.mat')['inputValue'].shape
angel_pos =  sio.loadmat('./new_data/angle_pos.mat')['angle_pos'].shape
threeD_pos = sio.loadmat('./new_data/threeD_pos.mat')['threeD_pos'].shape
print(inputValue, angel_pos, threeD_pos)
# print(threeD_pos)