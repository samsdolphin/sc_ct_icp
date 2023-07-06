# SC-CT-ICP

### Run the code
execute `roslaunch sc_ct_icp sc_ct_icp.launch`:
* node `pub_pcd` is used to read `.bin` KITTI point cloud, poses and corresponding time sequences and publish it to the scan context node below. 
* node `SC_PGO` is the scan context node which will do the loop detection and closure work.
* node `log_sc_cticp` is to log the loop closured poses.

### Notice
in the repository, we don't provide the code of ct_icp since I assume you have the poses and point clouds prepared.