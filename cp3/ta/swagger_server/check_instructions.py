import instruction_db
import os

db=instruction_db.InstructionDB(os.expanduser('~/catkin_ws/src/cp3_base/instructions/instructions-all.json'))
utilities=['favor-timeliness', 'favor-safety', 'favor-efficiency']
configs=['amcl-kinect', 'amcl-lidar', 'mrpt-kinect', 'mrpt-lidar', 'aruco-camera']

num = 0
numbad = 0
for i in range(59):
    for j in range(59):
        if i == j: continue
        for u in utilities:
            for c in configs:
                path=db.get_path('l'+i, 'l'+j, '%s-%s' %(u,c))
                if path is None: continue
                num=num+1
                if len(path)<2:
                    print('l%s->l%s has an empty path (no plan) in configuration %s under utility %s' %(i,j,c,u))
                    numbad=numbad+1

print('There are a total of %s empty paths out of %s' %(numbad,num))