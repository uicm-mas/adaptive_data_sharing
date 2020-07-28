#固定均匀的PER下，运行算法50次
for((i = 0; i< 50; i++));do
    roslaunch data_sharing eager_asyn.launch;
    sleep 0.2;
done

