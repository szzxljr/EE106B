import numpy as np

pos_array = np.array([[ 0.10615215,  0.06610176,  0.02687931, -0.01354061],
       [ 0.14153885,  0.10766521,  0.07718191,  0.05674296],
       [ 0.9       ,  0.9       ,  0.9       ,  0.9       ],
       [ 1.        ,  1.        ,  1.        ,  1.        ]])
delta_t_list = np.array([0.03615403175354004, 0.03181290626525879, 0.032000064849853516])

print('pos_array',pos_array)
print('delta_t_list',delta_t_list)
# pos_in_space = np.dot(g_base_camera,pos_array)
vx,vy,vz = 0.0,0.0,0.0
for n in range(3):
	vx += (1.0/3)*(pos_array[0,n+1]-pos_array[0,n])/delta_t_list[n]
	vy += (1.0/3)*(pos_array[1,n+1]-pos_array[1,n])/delta_t_list[n]
	vz += (1.0/3)*(pos_array[2,n+1]-pos_array[2,n])/delta_t_list[n]
	print('vx',vx)
	print('vy',vy)
	print('vz',vz)
