uint32 to_idx
uint32 from_idx

# relative pose from from_idx to to_idx
geometry_msgs/Pose transform

# information matrix expressed in target frame (right-multiplied), upper triangular
float32 inf_mat_px_px
float32 inf_mat_px_py
float32 inf_mat_px_pz
float32 inf_mat_px_ox
float32 inf_mat_px_oy
float32 inf_mat_px_oz
float32 inf_mat_py_py
float32 inf_mat_py_pz
float32 inf_mat_py_ox
float32 inf_mat_py_oy
float32 inf_mat_py_oz
float32 inf_mat_pz_pz
float32 inf_mat_pz_ox
float32 inf_mat_pz_oy
float32 inf_mat_pz_oz
float32 inf_mat_ox_ox
float32 inf_mat_ox_oy
float32 inf_mat_ox_oz
float32 inf_mat_oy_oy
float32 inf_mat_oy_oz
float32 inf_mat_oz_oz
