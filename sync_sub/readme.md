# ROSで複数のトピックから同期的にメッセージを受け取る   
https://qiita.com/nabion/items/319d4ffdc3d87bfb0076



```
cd ~/catkin_ws/src

catkin_create_pkg sync_sub std_msgs rospy roscpp
```

を行ってからできるディレクトリにcloneする

cloneしたらcatkin_wsでcatkin_makeすると使えるようになる
