#!/usr/bin/env python

import numpy as np
from scipy.optimize import minimize

# ROS msgs
import rclpy

# ROS types
from arp_types.srv import LoadMesh
from arp_types.srv import SingleShotPath
from arp_msgs.srv import GenerateRasterToolPath

# Point Control
TEST_MODELPACK_FILEPATH = "path/to/mesh/modelpack"

# Takes in the thickness vector and calculates the
# scalar error
def error_func (thickness_vec):
    #Calculates the standard deviation thickness
    return np.std(thickness_vec)

# Cals mesh constructor via a client call to the LoadMesh func
def call_mesh(node):

    #Construct the client
    cli_mesh_tpp = node.create_client(LoadMesh, "load_mesh")

    # Build Mesh request
    req_mesh = LoadMesh.Request()
    req_mesh.filepath = TEST_MODELPACK_FILEPATH

    # Call Mesh service
    while not cli_mesh_tpp.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Modelpack Service not avaliable, waiting again...")
    
    future_mesh = cli_mesh_tpp.call_async(req_mesh)
    rclpy.spin_until_future_complete(node, future_mesh)

    # Unpack Mesh result
    mesh_result = future_mesh.result()

    # Return Mesh
    return mesh_result.mesh


# Calls noether via a client call to the noether caller
def call_noether(line_spacing, point_spacing, tool_liftoff,
                 min_hole_size, min_seg_size, search_radius, mesh, node):
    
    #Construct the client
    cli_noether_tpp = node.create_client(GenerateRasterToolPath, "genrate_raster_tool_path")

    
    # Build Noether Request
    req_noether = GenerateRasterToolPath.Request()
    req_noether.mesh_frame = "world"
    req_noether.mesh = mesh
    req_noether.mesh_as_file_flag = False

    req_noether.line_spacing = line_spacing
    req_noether.point_spacing = point_spacing
    req_noether.tool_liftoff = tool_liftoff
    
    req_noether.min_hole_size = min_hole_size
    req_noether.min_segment_size = min_seg_size
    req_noether.search_radius = search_radius

    # Call Noether service
    while not cli_noether_tpp.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Noether Service not avaliable, waiting again...")

    future_noether = cli_noether_tpp.call_async(req_noether)
    rclpy.spin_until_future_complete(node, future_noether)

    # Unpack Noether result
    noether_result = future_noether.result()

    return noether_result.tool_path.segments


    # Return the toolpath to be used via exleys' code, wrapped in
    # a tuple with the boolean sucess - this function may just be
    # represented in the final optmize code w/o a seperate call,
    # however for the purpose of layout it will be represented as a
    # seperate function right now

def call_thickness(toolpath, mesh, node):

    cli_mesh_calc = node.create_client(SingleShotPath, "calc_thickness_err")

    # Build SingleShotPath request
    req_thickness = SingleShotPath.Request()
    req_thickness.mesh = mesh
    # req_thickness.sample mesh // TODO ASK ABOUT THIS TO ADAM
    # req_thickness.pulse_calibration //TODO ASK ABOUT THIS TOO

    req_thickness.path = toolpath
    # req_thickness.timestamps // TODO FIGURE OUT HOW WE WANNA SET THESE (const?)

    # REQUIRES
    # Mesh collision mesh
    # SampledMesh sample_mesh : mesh and points w/ positions, normals, areas, parent_faces, materials
    # PulseCalibrationParams pulse_calibration

    # geometry_msgs/PoseArray[] path
    # float32[] timestamps

    # RETURNS
    # float32 loss
    # float32[] thicknesses
    # bool sucess
    # string message

    # Call SingleShotPath
    while not cli_mesh_calc.wait_for_service(timeout_sec=1.0):
            node.get_logger().info("SingleShotPath Service not avaliable, waiting again...")
    
    future_calc = cli_mesh_calc.call_async(req_thickness)
    rclpy.spin_until_future_complete(node, future_calc)

    # Unpack SingleShotPath result
    ssp_result = future_calc.result()

    # Return thickness
    return ssp_result

def optimize_thickness(v0, min_seg_size, min_hole_size, search_radius, node):

    mesh = call_mesh(node)
    toolpath = call_noether(v0[0], v0[1], v0[2], min_hole_size, min_seg_size, search_radius, mesh, node)
    thickness_vec = call_thickness(toolpath, mesh, node)

    ## TODO Add buffers and error checks for whether or not the client node was sucessful to returns (tuple form)

    return error_func(thickness_vec.thicknesses)

def main(args=None):

    rclpy.init(args)
    node = rclpy.create_node("scipy_optimize_client")

    # Start of the variables that we wish to optimize
    line_spacing = 0.5 # Or roughly ~1/2 the fan width
    point_spaing = 0.5 # Or roguhly ~1/2 the minor axis of the above
    liftoff = 0.1

    # The variables we wish to pass as constant
    min_seg_size = 0.1
    min_hole_size = 0.1
    search_radius = 0.1

    v0 = [line_spacing, point_spaing, liftoff]
    # Not entirley sure what method we wer interested in using for the calation - left as TODO for now!
    res = minimize(optimize_thickness, v0, method='TODO',
                   args=(min_seg_size, min_hole_size, search_radius, node), options={'xatol': 1e-8, 'disp': True})
    
    print(res)
    
    # Destroy node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
